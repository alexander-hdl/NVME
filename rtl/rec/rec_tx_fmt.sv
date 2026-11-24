// =============================================================================
// rec_tx_fmt.sv — text formatter of the recorder: messages -> bytes -> UART (WP1).
//
// Domain  : link_clk (200 MHz).  Spec: STAND/RECORDER_SPEC.md 2.3-2.6, 7.2, 4.4 (WP1).
// Ports   : msg_valid/msg_ready  one message per handshake (msg_ready is a register, high only
//                                while the formatter is idle).  On acceptance msg_kind, msg_code,
//                                msg_arg0/1 and the whole `snap` are copied: rec_ctrl may change
//                                them at once.
//           tel_tick             1-clock pulse: a `$T` line is due.  It is remembered (one level,
//                                ticks during a line merge into one) and sent when the formatter
//                                is idle and no message is waiting.
//           txd                  serial output (register inside uart_tx).
// Priority: a waiting message (reply or event; their order is decided by rec_ctrl) always goes
//           before a pending tel_tick.  A line is never interrupted: the formatter takes the next
//           message only after the whole line (up to and including LF) is in the 2048-byte FIFO.
//           If the FIFO is nearly full (>= 2040 bytes) the formatter waits.
// Format  : every line ends with LF.  Numbers are upper-case hex of fixed width (zero padded; a
//           source wider than its field is cut to the low digits).  Layout of every message is
//           the template ROM below (one element = literal text | field {key, width, source}).
//
// msg_kind (rec_msg_kind_e) / msg_code -> line:
//   MK_OK + opcode      OK <WORD> <fields>; the value fields come from msg_arg0/1 (echo of the value
//                       just set) or from snap (CFG?, SMART?, DDR?, PROF?, PING cal/uok):
//     OP_PING     OK PING ver=REC_VER bld=BLD_DATE (parameter) cal uok(snap)
//     OP_STAT / OP_RESQ / unknown opcode   OK            (after the $T / !R* line sent by rec_ctrl)
//     OP_CFGQ     OK CFG md bp stp len sid ddr tel sm pn pr pe rsid       (snap md, cfg_*)
//     OP_TEL      OK TEL tel=arg0[15:0]        OP_SMART  OK SMART sm=arg0[15:0]
//     OP_SMARTQ   OK SMART cw tk sp us dur duw pc poh uns me wt ct t1 ok er st rtd
//                   (snap.smart, ok=snap.smo, er=snap.sme, st=arg0[15:0] log status, rtd=snap.rtd)
//     OP_MODE     OK MODE bp=arg0[3:0]         OP_RATE   OK RATE stp=arg0[31:0]
//     OP_LEN      OK LEN len=arg0[31:0]        OP_SID    OK SID sid=arg0[31:0]
//     OP_DDRSZ    OK DDRSZ ddr=arg0[15:0]
//     OP_PROF_CLR OK PROF pn=arg0[7:0]         OP_PROF_ADD/SET  OK PROF i=arg0[7:0]
//     OP_PROF_REP OK PROF pr=arg0[15:0]        OP_PROF_END      OK PROF pe=arg0[3:0]
//     OP_PROFQ    OK PROF pn pr pe (snap.cfg_pn/pr/pe)
//     OP_REC / OP_REC_PROF  OK REC sid=arg0[31:0]
//     OP_STOP / OP_PFAIL / OP_ABORT / OP_FORMAT / OP_MON / OP_DTEST / OP_READ_LAST / OP_READ_N /
//     OP_READ_END    OK <WORD>        OP_READ_SID  OK READ rsid=arg0[31:0]
//     OP_MEMTEST  OK MEMTEST mib=arg0[15:0]   OP_DDRQ  OK DDR cal ddr ddrm mte mta (snap)
//   MK_ERR            ERR <msg_code>
//   MK_PROF_LINE      P i=arg0[63:56] stp=arg0[31:0] dur=arg1[39:0]
//   MK_TEL            $T line from snap (STAT?)       (tel_tick gives the same line)
//   MK_CFGQ / MK_SMARTQ / MK_DDRQ   same line as MK_OK + OP_CFGQ / OP_SMARTQ / OP_DDRQ
//   MK_EV_B  !B ver cal uok (snap)
//   MK_EV_S  !S st=arg0[3:0] md=arg0[7:4] t=arg1[31:0]
//   MK_EV_E  !E ce=arg0[7:0] cd=arg0[15:8] fl=arg0[23:16] st=arg0[27:24]
//   MK_EV_RR !RR ...  MK_EV_RP !RP ...  MK_EV_RV !RV ...  (all from snap)
//   MK_EV_RF !RF ms=arg0[31:0] ok=arg1[3:0]      MK_EV_RM !RM ... (snap)     MK_EV_RD !RD ... (snap)
//   MK_NONE / other   consumed, nothing is sent.
// =============================================================================
module rec_tx_fmt
  import rec_pkg::*;
#(
  parameter int CLK_HZ = 200_000_000,
  parameter int BAUD   = 921_600,
  parameter logic [31:0] BLD_DATE = 32'h2026_1124   // PING bld: yyyymmdd as hex digits
) (
  input  logic        clk,
  input  logic        rst,
  input  logic        msg_valid,   // link
  output logic        msg_ready,   // link
  input  logic [4:0]  msg_kind,    // link, rec_msg_kind_e
  input  logic [7:0]  msg_code,    // link
  input  logic [63:0] msg_arg0,    // link
  input  logic [63:0] msg_arg1,    // link
  input  rec_snap_t   snap,        // link
  input  logic        tel_tick,    // link
  output logic        txd          // pin
);
  // ---------------------------------------------------------------- template tables
  localparam logic [1:0] K_END = 2'd0, K_TXT = 2'd1, K_FLD = 2'd2;
  // element: {kind[1:0], text[79:0], src[7:0], width[4:0], 1'b0}.  text is right aligned
  // (a string literal), leading zero bytes are skipped; FLD prints ' ' + text + '=' (when text
  // is not empty) + `width` hex digits of the source.
  function automatic logic [95:0] e_txt(input logic [79:0] t);
    return {K_TXT, t, 8'd0, 5'd0, 1'b0};
  endfunction
  function automatic logic [95:0] e_fld(input logic [79:0] t, input int w, input int s);
    return {K_FLD, t, 8'(s), 5'(w), 1'b0};
  endfunction
  localparam logic [95:0] E_END_EL = {K_END, 94'd0};

  // message copy (taken at acceptance)
  rec_snap_t    snap_q;
  logic [7:0]   code_q;
  logic [63:0]  arg0_q, arg1_q;

  // ---- source indices (GENERATED; one entry per distinct value used by a template)
  localparam int SRC_CONST_VER = 0;
  localparam int SRC_CODE = 1;
  localparam int SRC_A0 = 2;
  localparam int SRC_A1 = 3;
  localparam int SRC_CONST_BLD = 4;
  localparam int SRC_A0_I = 5;
  localparam int SRC_EVS_ST = 6;
  localparam int SRC_EVS_MD = 7;
  localparam int SRC_EVE_CE = 8;
  localparam int SRC_EVE_CD = 9;
  localparam int SRC_EVE_FL = 10;
  localparam int SRC_EVE_ST = 11;
  localparam int SRC_S_T_MS = 12;
  localparam int SRC_S_ST = 13;
  localparam int SRC_S_MD = 14;
  localparam int SRC_S_COK = 15;
  localparam int SRC_S_UOK = 16;
  localparam int SRC_S_CS = 17;
  localparam int SRC_S_CE = 18;
  localparam int SRC_S_CD = 19;
  localparam int SRC_S_FL = 20;
  localparam int SRC_S_LK = 21;
  localparam int SRC_S_CAL = 22;
  localparam int SRC_S_STP = 23;
  localparam int SRC_S_SEG = 24;
  localparam int SRC_S_REP = 25;
  localparam int SRC_S_SRC = 26;
  localparam int SRC_S_WR = 27;
  localparam int SRC_S_DRP = 28;
  localparam int SRC_S_DDR = 29;
  localparam int SRC_S_DDRM = 30;
  localparam int SRC_S_BPM = 31;
  localparam int SRC_S_RNG = 32;
  localparam int SRC_S_SSD = 33;
  localparam int SRC_S_CMD = 34;
  localparam int SRC_S_CER = 35;
  localparam int SRC_S_LAT = 36;
  localparam int SRC_S_VB = 37;
  localparam int SRC_S_VE = 38;
  localparam int SRC_S_TK = 39;
  localparam int SRC_S_CW = 40;
  localparam int SRC_S_SMO = 41;
  localparam int SRC_S_SME = 42;
  localparam int SRC_S_XT = 43;
  localparam int SRC_S_XVI = 44;
  localparam int SRC_S_XVA = 45;
  localparam int SRC_S_XVB = 46;
  localparam int SRC_S_CFG_BP = 47;
  localparam int SRC_S_CFG_STP = 48;
  localparam int SRC_S_CFG_LEN = 49;
  localparam int SRC_S_CFG_SID = 50;
  localparam int SRC_S_CFG_DDR = 51;
  localparam int SRC_S_CFG_TEL = 52;
  localparam int SRC_S_CFG_SM = 53;
  localparam int SRC_S_CFG_PN = 54;
  localparam int SRC_S_CFG_PR = 55;
  localparam int SRC_S_CFG_PE = 56;
  localparam int SRC_S_CFG_RSID = 57;
  localparam int SRC_S_END_FOUND = 58;
  localparam int SRC_S_MTE = 59;
  localparam int SRC_S_MTA = 60;
  localparam int SRC_S_MT_BYTES = 61;
  localparam int SRC_S_DT_HE = 62;
  localparam int SRC_S_DT_WR = 63;
  localparam int SRC_S_PAD_BLOCKS = 64;
  localparam int SRC_S_TRUNC_BLOCKS = 65;
  localparam int SRC_S_DUR = 66;
  localparam int SRC_S_SID = 67;
  localparam int SRC_S_RTD = 68;
  localparam int SRC_S_RP_SRC = 69;
  localparam int SRC_S_RP_INB = 70;
  localparam int SRC_S_RP_LAT = 71;
  localparam int SRC_S_RP_LOV = 72;
  localparam int SRC_S_RP_PND = 73;
  localparam int SRC_S_RP_PDD = 74;
  localparam int SRC_S_RP_PRG = 75;
  localparam int SRC_S_RP_PAD = 76;
  localparam int SRC_S_RP_T1 = 77;
  localparam int SRC_S_RP_T2 = 78;
  localparam int SRC_S_RP_T3 = 79;
  localparam int SRC_S_RP_T4 = 80;
  localparam int SRC_S_RP_T5 = 81;
  localparam int SRC_S_RP_T6 = 82;
  localparam int SRC_S_RP_OK = 83;
  localparam int SRC_S_RP_LPOS = 84;
  localparam int SRC_SM_CW = 85;
  localparam int SRC_SM_TK = 86;
  localparam int SRC_SM_SP = 87;
  localparam int SRC_SM_SP_USED = 88;
  localparam int SRC_SM_DUR = 89;
  localparam int SRC_SM_DUW = 90;
  localparam int SRC_SM_PC = 91;
  localparam int SRC_SM_POH = 92;
  localparam int SRC_SM_UNS = 93;
  localparam int SRC_SM_ME = 94;
  localparam int SRC_SM_WT = 95;
  localparam int SRC_SM_CT = 96;
  localparam int SRC_SM_T1 = 97;
  localparam int SRC_V_VB = 98;
  localparam int SRC_V_VE = 99;
  localparam int SRC_V_OK = 100;
  localparam int SRC_V_BAD = 101;
  localparam int SRC_V_CRC = 102;
  localparam int SRC_V_PE = 103;
  localparam int SRC_V_OOO = 104;
  localparam int SRC_V_PRT = 105;
  localparam int SRC_V_FBP = 106;
  localparam int SRC_V_MVP = 107;
  localparam int SRC_V_LSQ = 108;
  localparam int SRC_V_LTS = 109;
  localparam int SRC_V_LST = 110;
  localparam int SRC_V_PAD = 111;
  localparam int SRC_V_FEP = 112;
  localparam int SRC_V_FEB = 113;
  localparam int SRC_V_EXP_DW = 114;
  localparam int SRC_V_GOT_DW = 115;
  localparam int SRC_V_OVF = 116;
  localparam int SRC_V_LFL = 117;   // WP8: !RV lfl = flags of the max-valid block
  localparam int NSRC = 118;
  localparam int T_OK           = 0;   // OK
  localparam int T_ERR          = 1;   // ERR hh
  localparam int T_OK_PING      = 2;   // OK PING ver bld cal uok
  localparam int T_OK_CFG       = 3;   // OK CFG ...
  localparam int T_OK_TEL       = 4;   // OK TEL tel
  localparam int T_OK_SMART     = 5;   // OK SMART sm
  localparam int T_OK_SMARTQ    = 6;   // OK SMART cw tk sp us dur duw pc poh uns me wt ct t1 ok er st rtd
  localparam int T_OK_MODE      = 7;   // OK MODE bp
  localparam int T_OK_RATE      = 8;   // OK RATE stp
  localparam int T_OK_LEN       = 9;   // OK LEN len
  localparam int T_OK_SID       = 10;   // OK SID sid
  localparam int T_OK_DDRSZ     = 11;   // OK DDRSZ ddr
  localparam int T_OK_PROF_PN   = 12;   // OK PROF pn
  localparam int T_OK_PROF_I    = 13;   // OK PROF i
  localparam int T_OK_PROF_PR   = 14;   // OK PROF pr
  localparam int T_OK_PROF_PE   = 15;   // OK PROF pe
  localparam int T_OK_PROFQ     = 16;   // OK PROF pn pr pe
  localparam int T_OK_REC       = 17;   // OK REC sid
  localparam int T_OK_STOP      = 18;   // OK STOP
  localparam int T_OK_PFAIL     = 19;   // OK PFAIL
  localparam int T_OK_ABORT     = 20;   // OK ABORT
  localparam int T_OK_READ      = 21;   // OK READ
  localparam int T_OK_READ_SID  = 22;   // OK READ rsid
  localparam int T_OK_FORMAT    = 23;   // OK FORMAT
  localparam int T_OK_MON       = 24;   // OK MON
  localparam int T_OK_MEMTEST   = 25;   // OK MEMTEST mib
  localparam int T_OK_DTEST     = 26;   // OK DTEST
  localparam int T_OK_DDR       = 27;   // OK DDR cal ddr ddrm mte mta
  localparam int T_P_LINE       = 28;   // P i stp dur
  localparam int T_TEL          = 29;   // $T ...
  localparam int T_EV_B         = 30;   // !B ver cal uok
  localparam int T_EV_S         = 31;   // !S st md t
  localparam int T_EV_E         = 32;   // !E ce cd fl st
  localparam int T_EV_RR        = 33;   // !RR sid wr drp pad trc ddrm ssd cmd cer lat dur
  localparam int T_EV_RP        = 34;   // !RP (7.2)
  localparam int T_EV_RV        = 35;   // !RV (2.6)
  localparam int T_EV_RF        = 36;   // !RF ms ok
  localparam int T_EV_RM        = 37;   // !RM byt err adr dur
  localparam int T_EV_RD        = 38;   // !RD wr drp he dur
  localparam int NT = 39;
  localparam int ROM_N = 243;
  localparam logic [95:0] ROM [0:ROM_N-1] = '{
    // T_OK: OK
    e_txt("OK"),
    E_END_EL,
    // T_ERR: ERR hh
    e_txt("ERR"),
    e_fld("", 2, SRC_CODE),
    E_END_EL,
    // T_OK_PING: OK PING ver bld cal uok
    e_txt("OK PING"),
    e_fld("ver", 2, SRC_CONST_VER),
    e_fld("bld", 8, SRC_CONST_BLD),
    e_fld("cal", 1, SRC_S_CAL),
    e_fld("uok", 1, SRC_S_UOK),
    E_END_EL,
    // T_OK_CFG: OK CFG ...
    e_txt("OK CFG"),
    e_fld("md", 1, SRC_S_MD),
    e_fld("bp", 1, SRC_S_CFG_BP),
    e_fld("stp", 8, SRC_S_CFG_STP),
    e_fld("len", 8, SRC_S_CFG_LEN),
    e_fld("sid", 8, SRC_S_CFG_SID),
    e_fld("ddr", 4, SRC_S_CFG_DDR),
    e_fld("tel", 4, SRC_S_CFG_TEL),
    e_fld("sm", 4, SRC_S_CFG_SM),
    e_fld("pn", 2, SRC_S_CFG_PN),
    e_fld("pr", 4, SRC_S_CFG_PR),
    e_fld("pe", 1, SRC_S_CFG_PE),
    e_fld("rsid", 8, SRC_S_CFG_RSID),
    E_END_EL,
    // T_OK_TEL: OK TEL tel
    e_txt("OK TEL"),
    e_fld("tel", 4, SRC_A0),
    E_END_EL,
    // T_OK_SMART: OK SMART sm
    e_txt("OK SMART"),
    e_fld("sm", 4, SRC_A0),
    E_END_EL,
    // T_OK_SMARTQ: OK SMART cw tk sp us dur duw pc poh uns me wt ct t1 ok er st rtd
    e_txt("OK SMART"),
    e_fld("cw", 2, SRC_SM_CW),
    e_fld("tk", 4, SRC_SM_TK),
    e_fld("sp", 2, SRC_SM_SP),
    e_fld("us", 2, SRC_SM_SP_USED),
    e_fld("dur", 16, SRC_SM_DUR),
    e_fld("duw", 16, SRC_SM_DUW),
    e_fld("pc", 8, SRC_SM_PC),
    e_fld("poh", 8, SRC_SM_POH),
    e_fld("uns", 8, SRC_SM_UNS),
    e_fld("me", 8, SRC_SM_ME),
    e_fld("wt", 8, SRC_SM_WT),
    e_fld("ct", 8, SRC_SM_CT),
    e_fld("t1", 4, SRC_SM_T1),
    e_fld("ok", 4, SRC_S_SMO),
    e_fld("er", 4, SRC_S_SME),
    e_fld("st", 4, SRC_A0),
    e_fld("rtd", 8, SRC_S_RTD),
    E_END_EL,
    // T_OK_MODE: OK MODE bp
    e_txt("OK MODE"),
    e_fld("bp", 1, SRC_A0),
    E_END_EL,
    // T_OK_RATE: OK RATE stp
    e_txt("OK RATE"),
    e_fld("stp", 8, SRC_A0),
    E_END_EL,
    // T_OK_LEN: OK LEN len
    e_txt("OK LEN"),
    e_fld("len", 8, SRC_A0),
    E_END_EL,
    // T_OK_SID: OK SID sid
    e_txt("OK SID"),
    e_fld("sid", 8, SRC_A0),
    E_END_EL,
    // T_OK_DDRSZ: OK DDRSZ ddr
    e_txt("OK DDRSZ"),
    e_fld("ddr", 4, SRC_A0),
    E_END_EL,
    // T_OK_PROF_PN: OK PROF pn
    e_txt("OK PROF"),
    e_fld("pn", 2, SRC_A0),
    E_END_EL,
    // T_OK_PROF_I: OK PROF i
    e_txt("OK PROF"),
    e_fld("i", 2, SRC_A0),
    E_END_EL,
    // T_OK_PROF_PR: OK PROF pr
    e_txt("OK PROF"),
    e_fld("pr", 4, SRC_A0),
    E_END_EL,
    // T_OK_PROF_PE: OK PROF pe
    e_txt("OK PROF"),
    e_fld("pe", 1, SRC_A0),
    E_END_EL,
    // T_OK_PROFQ: OK PROF pn pr pe
    e_txt("OK PROF"),
    e_fld("pn", 2, SRC_S_CFG_PN),
    e_fld("pr", 4, SRC_S_CFG_PR),
    e_fld("pe", 1, SRC_S_CFG_PE),
    E_END_EL,
    // T_OK_REC: OK REC sid
    e_txt("OK REC"),
    e_fld("sid", 8, SRC_A0),
    E_END_EL,
    // T_OK_STOP: OK STOP
    e_txt("OK STOP"),
    E_END_EL,
    // T_OK_PFAIL: OK PFAIL
    e_txt("OK PFAIL"),
    E_END_EL,
    // T_OK_ABORT: OK ABORT
    e_txt("OK ABORT"),
    E_END_EL,
    // T_OK_READ: OK READ
    e_txt("OK READ"),
    E_END_EL,
    // T_OK_READ_SID: OK READ rsid
    e_txt("OK READ"),
    e_fld("rsid", 8, SRC_A0),
    E_END_EL,
    // T_OK_FORMAT: OK FORMAT
    e_txt("OK FORMAT"),
    E_END_EL,
    // T_OK_MON: OK MON
    e_txt("OK MON"),
    E_END_EL,
    // T_OK_MEMTEST: OK MEMTEST mib
    e_txt("OK MEMTEST"),
    e_fld("mib", 4, SRC_A0),
    E_END_EL,
    // T_OK_DTEST: OK DTEST
    e_txt("OK DTEST"),
    E_END_EL,
    // T_OK_DDR: OK DDR cal ddr ddrm mte mta
    e_txt("OK DDR"),
    e_fld("cal", 1, SRC_S_CAL),
    e_fld("ddr", 8, SRC_S_DDR),
    e_fld("ddrm", 8, SRC_S_DDRM),
    e_fld("mte", 8, SRC_S_MTE),
    e_fld("mta", 8, SRC_S_MTA),
    E_END_EL,
    // T_P_LINE: P i stp dur
    e_txt("P"),
    e_fld("i", 2, SRC_A0_I),
    e_fld("stp", 8, SRC_A0),
    e_fld("dur", 10, SRC_A1),
    E_END_EL,
    // T_TEL: $T ...
    e_txt("$T"),
    e_fld("v", 2, SRC_CONST_VER),
    e_fld("t", 8, SRC_S_T_MS),
    e_fld("st", 1, SRC_S_ST),
    e_fld("md", 1, SRC_S_MD),
    e_fld("cok", 1, SRC_S_COK),
    e_fld("uok", 1, SRC_S_UOK),
    e_fld("cs", 1, SRC_S_CS),
    e_fld("ce", 2, SRC_S_CE),
    e_fld("cd", 2, SRC_S_CD),
    e_fld("fl", 2, SRC_S_FL),
    e_fld("lk", 1, SRC_S_LK),
    e_fld("cal", 1, SRC_S_CAL),
    e_fld("stp", 8, SRC_S_STP),
    e_fld("seg", 2, SRC_S_SEG),
    e_fld("rep", 4, SRC_S_REP),
    e_fld("src", 12, SRC_S_SRC),
    e_fld("wr", 12, SRC_S_WR),
    e_fld("drp", 8, SRC_S_DRP),
    e_fld("ddr", 8, SRC_S_DDR),
    e_fld("ddrm", 8, SRC_S_DDRM),
    e_fld("bpm", 10, SRC_S_BPM),
    e_fld("rng", 8, SRC_S_RNG),
    e_fld("ssd", 16, SRC_S_SSD),
    e_fld("cmd", 8, SRC_S_CMD),
    e_fld("cer", 8, SRC_S_CER),
    e_fld("lat", 8, SRC_S_LAT),
    e_fld("vb", 12, SRC_S_VB),
    e_fld("ve", 8, SRC_S_VE),
    e_fld("tk", 4, SRC_S_TK),
    e_fld("cw", 2, SRC_S_CW),
    e_fld("smo", 4, SRC_S_SMO),
    e_fld("sme", 4, SRC_S_SME),
    e_fld("xt", 3, SRC_S_XT),
    e_fld("xvi", 3, SRC_S_XVI),
    e_fld("xva", 3, SRC_S_XVA),
    e_fld("xvb", 3, SRC_S_XVB),
    E_END_EL,
    // T_EV_B: !B ver cal uok
    e_txt("!B"),
    e_fld("ver", 2, SRC_CONST_VER),
    e_fld("cal", 1, SRC_S_CAL),
    e_fld("uok", 1, SRC_S_UOK),
    E_END_EL,
    // T_EV_S: !S st md t
    e_txt("!S"),
    e_fld("st", 1, SRC_EVS_ST),
    e_fld("md", 1, SRC_EVS_MD),
    e_fld("t", 8, SRC_A1),
    E_END_EL,
    // T_EV_E: !E ce cd fl st
    e_txt("!E"),
    e_fld("ce", 2, SRC_EVE_CE),
    e_fld("cd", 2, SRC_EVE_CD),
    e_fld("fl", 2, SRC_EVE_FL),
    e_fld("st", 1, SRC_EVE_ST),
    E_END_EL,
    // T_EV_RR: !RR sid wr drp pad trc ddrm ssd cmd cer lat dur
    e_txt("!RR"),
    e_fld("sid", 8, SRC_S_SID),
    e_fld("wr", 12, SRC_S_WR),
    e_fld("drp", 8, SRC_S_DRP),
    e_fld("pad", 8, SRC_S_PAD_BLOCKS),
    e_fld("trc", 8, SRC_S_TRUNC_BLOCKS),
    e_fld("ddrm", 8, SRC_S_DDRM),
    e_fld("ssd", 16, SRC_S_SSD),
    e_fld("cmd", 8, SRC_S_CMD),
    e_fld("cer", 8, SRC_S_CER),
    e_fld("lat", 8, SRC_S_LAT),
    e_fld("dur", 16, SRC_S_DUR),
    E_END_EL,
    // T_EV_RP: !RP (7.2)
    e_txt("!RP"),
    e_fld("src", 1, SRC_S_RP_SRC),
    e_fld("inb", 1, SRC_S_RP_INB),
    e_fld("lat", 1, SRC_S_RP_LAT),
    e_fld("lov", 1, SRC_S_RP_LOV),
    e_fld("pnd", 12, SRC_S_RP_PND),
    e_fld("pdd", 8, SRC_S_RP_PDD),
    e_fld("prg", 8, SRC_S_RP_PRG),
    e_fld("pad", 8, SRC_S_RP_PAD),
    e_fld("t1", 16, SRC_S_RP_T1),
    e_fld("t2", 16, SRC_S_RP_T2),
    e_fld("t3", 16, SRC_S_RP_T3),
    e_fld("t4", 16, SRC_S_RP_T4),
    e_fld("t5", 16, SRC_S_RP_T5),
    e_fld("t6", 16, SRC_S_RP_T6),
    e_fld("ok", 1, SRC_S_RP_OK),
    e_fld("rtd", 8, SRC_S_RTD),
    e_fld("ssd", 16, SRC_S_SSD),
    e_fld("lpos", 10, SRC_S_RP_LPOS),
    E_END_EL,
    // T_EV_RV: !RV (2.6)
    e_txt("!RV"),
    e_fld("vb", 12, SRC_V_VB),
    e_fld("ok", 10, SRC_V_OK),
    e_fld("bad", 8, SRC_V_BAD),
    e_fld("crc", 8, SRC_V_CRC),
    e_fld("pe", 8, SRC_V_PE),
    e_fld("ooo", 8, SRC_V_OOO),
    e_fld("prt", 8, SRC_V_PRT),
    e_fld("fbp", 10, SRC_V_FBP),
    e_fld("mvp", 10, SRC_V_MVP),
    e_fld("lsq", 16, SRC_V_LSQ),
    e_fld("lts", 16, SRC_V_LTS),
    e_fld("lst", 8, SRC_V_LST),
    e_fld("pad", 8, SRC_V_PAD),
    e_fld("fep", 10, SRC_V_FEP),
    e_fld("feb", 2, SRC_V_FEB),
    e_fld("exp", 8, SRC_V_EXP_DW),
    e_fld("got", 8, SRC_V_GOT_DW),
    e_fld("end", 1, SRC_S_END_FOUND),
    e_fld("ovf", 1, SRC_V_OVF),
    e_fld("lfl", 2, SRC_V_LFL),   // WP8
    e_fld("dur", 16, SRC_S_DUR),
    E_END_EL,
    // T_EV_RF: !RF ms ok
    e_txt("!RF"),
    e_fld("ms", 8, SRC_A0),
    e_fld("ok", 1, SRC_A1),
    E_END_EL,
    // T_EV_RM: !RM byt err adr dur
    e_txt("!RM"),
    e_fld("byt", 8, SRC_S_MT_BYTES),
    e_fld("err", 8, SRC_S_MTE),
    e_fld("adr", 8, SRC_S_MTA),
    e_fld("dur", 16, SRC_S_DUR),
    E_END_EL,
    // T_EV_RD: !RD wr drp he dur
    e_txt("!RD"),
    e_fld("wr", 12, SRC_S_DT_WR),
    e_fld("drp", 8, SRC_S_DRP),
    e_fld("he", 8, SRC_S_DT_HE),
    e_fld("dur", 16, SRC_S_DUR),
    E_END_EL
  };

  // start address of template t = index after the t-th END element
  function automatic int rom_base(input int t);
    int idx, seen;
    idx = 0; seen = 0;
    while (seen < t) begin
      if (ROM[idx][95:94] == K_END) seen++;
      idx++;
    end
    return idx;
  endfunction
  localparam int BASE [0:NT-1] = '{rom_base(0), rom_base(1), rom_base(2), rom_base(3), rom_base(4), rom_base(5), rom_base(6), rom_base(7), rom_base(8), rom_base(9), rom_base(10), rom_base(11), rom_base(12), rom_base(13), rom_base(14), rom_base(15), rom_base(16), rom_base(17), rom_base(18), rom_base(19), rom_base(20), rom_base(21), rom_base(22), rom_base(23), rom_base(24), rom_base(25), rom_base(26), rom_base(27), rom_base(28), rom_base(29), rom_base(30), rom_base(31), rom_base(32), rom_base(33), rom_base(34), rom_base(35), rom_base(36), rom_base(37), rom_base(38)};


  // ---- source mux (GENERATED): value of source `idx`, zero extended to 64 bit
  function automatic logic [63:0] src_val(input int idx);
    logic [63:0] v;
    v = 64'd0;
    case (idx)
      SRC_CONST_VER         : v = 64'(REC_VER);
      SRC_CODE              : v = 64'(code_q);
      SRC_A0                : v = arg0_q;
      SRC_A1                : v = arg1_q;
      SRC_CONST_BLD         : v = 64'(BLD_DATE);
      SRC_A0_I              : v = 64'(arg0_q[63:56]);
      SRC_EVS_ST            : v = 64'(arg0_q[3:0]);
      SRC_EVS_MD            : v = 64'(arg0_q[7:4]);
      SRC_EVE_CE            : v = 64'(arg0_q[7:0]);
      SRC_EVE_CD            : v = 64'(arg0_q[15:8]);
      SRC_EVE_FL            : v = 64'(arg0_q[23:16]);
      SRC_EVE_ST            : v = 64'(arg0_q[27:24]);
      SRC_S_T_MS            : v = 64'(snap_q.t_ms);
      SRC_S_ST              : v = 64'(snap_q.st);
      SRC_S_MD              : v = 64'(snap_q.md);
      SRC_S_COK             : v = 64'(snap_q.cok);
      SRC_S_UOK             : v = 64'(snap_q.uok);
      SRC_S_CS              : v = 64'(snap_q.cs);
      SRC_S_CE              : v = 64'(snap_q.ce);
      SRC_S_CD              : v = 64'(snap_q.cd);
      SRC_S_FL              : v = 64'(snap_q.fl);
      SRC_S_LK              : v = 64'(snap_q.lk);
      SRC_S_CAL             : v = 64'(snap_q.cal);
      SRC_S_STP             : v = 64'(snap_q.stp);
      SRC_S_SEG             : v = 64'(snap_q.seg);
      SRC_S_REP             : v = 64'(snap_q.rep);
      SRC_S_SRC             : v = 64'(snap_q.src);
      SRC_S_WR              : v = 64'(snap_q.wr);
      SRC_S_DRP             : v = 64'(snap_q.drp);
      SRC_S_DDR             : v = 64'(snap_q.ddr);
      SRC_S_DDRM            : v = 64'(snap_q.ddrm);
      SRC_S_BPM             : v = 64'(snap_q.bpm);
      SRC_S_RNG             : v = 64'(snap_q.rng);
      SRC_S_SSD             : v = 64'(snap_q.ssd);
      SRC_S_CMD             : v = 64'(snap_q.cmd);
      SRC_S_CER             : v = 64'(snap_q.cer);
      SRC_S_LAT             : v = 64'(snap_q.lat);
      SRC_S_VB              : v = 64'(snap_q.vb);
      SRC_S_VE              : v = 64'(snap_q.ve);
      SRC_S_TK              : v = 64'(snap_q.tk);
      SRC_S_CW              : v = 64'(snap_q.cw);
      SRC_S_SMO             : v = 64'(snap_q.smo);
      SRC_S_SME             : v = 64'(snap_q.sme);
      SRC_S_XT              : v = 64'(snap_q.xt);
      SRC_S_XVI             : v = 64'(snap_q.xvi);
      SRC_S_XVA             : v = 64'(snap_q.xva);
      SRC_S_XVB             : v = 64'(snap_q.xvb);
      SRC_S_CFG_BP          : v = 64'(snap_q.cfg_bp);
      SRC_S_CFG_STP         : v = 64'(snap_q.cfg_stp);
      SRC_S_CFG_LEN         : v = 64'(snap_q.cfg_len);
      SRC_S_CFG_SID         : v = 64'(snap_q.cfg_sid);
      SRC_S_CFG_DDR         : v = 64'(snap_q.cfg_ddr);
      SRC_S_CFG_TEL         : v = 64'(snap_q.cfg_tel);
      SRC_S_CFG_SM          : v = 64'(snap_q.cfg_sm);
      SRC_S_CFG_PN          : v = 64'(snap_q.cfg_pn);
      SRC_S_CFG_PR          : v = 64'(snap_q.cfg_pr);
      SRC_S_CFG_PE          : v = 64'(snap_q.cfg_pe);
      SRC_S_CFG_RSID        : v = 64'(snap_q.cfg_rsid);
      SRC_S_END_FOUND       : v = 64'(snap_q.end_found);
      SRC_S_MTE             : v = 64'(snap_q.mte);
      SRC_S_MTA             : v = 64'(snap_q.mta);
      SRC_S_MT_BYTES        : v = 64'(snap_q.mt_bytes);
      SRC_S_DT_HE           : v = 64'(snap_q.dt_he);
      SRC_S_DT_WR           : v = 64'(snap_q.dt_wr);
      SRC_S_PAD_BLOCKS      : v = 64'(snap_q.pad_blocks);
      SRC_S_TRUNC_BLOCKS    : v = 64'(snap_q.trunc_blocks);
      SRC_S_DUR             : v = 64'(snap_q.dur);
      SRC_S_SID             : v = 64'(snap_q.sid);
      SRC_S_RTD             : v = 64'(snap_q.rtd);
      SRC_S_RP_SRC          : v = 64'(snap_q.rp_src);
      SRC_S_RP_INB          : v = 64'(snap_q.rp_inb);
      SRC_S_RP_LAT          : v = 64'(snap_q.rp_lat);
      SRC_S_RP_LOV          : v = 64'(snap_q.rp_lov);
      SRC_S_RP_PND          : v = 64'(snap_q.rp_pnd);
      SRC_S_RP_PDD          : v = 64'(snap_q.rp_pdd);
      SRC_S_RP_PRG          : v = 64'(snap_q.rp_prg);
      SRC_S_RP_PAD          : v = 64'(snap_q.rp_pad);
      SRC_S_RP_T1           : v = 64'(snap_q.rp_t1);
      SRC_S_RP_T2           : v = 64'(snap_q.rp_t2);
      SRC_S_RP_T3           : v = 64'(snap_q.rp_t3);
      SRC_S_RP_T4           : v = 64'(snap_q.rp_t4);
      SRC_S_RP_T5           : v = 64'(snap_q.rp_t5);
      SRC_S_RP_T6           : v = 64'(snap_q.rp_t6);
      SRC_S_RP_OK           : v = 64'(snap_q.rp_ok);
      SRC_S_RP_LPOS         : v = 64'(snap_q.rp_lpos);
      SRC_SM_CW             : v = 64'(snap_q.smart.cw);
      SRC_SM_TK             : v = 64'(snap_q.smart.tk);
      SRC_SM_SP             : v = 64'(snap_q.smart.sp);
      SRC_SM_SP_USED        : v = 64'(snap_q.smart.sp_used);
      SRC_SM_DUR            : v = 64'(snap_q.smart.dur);
      SRC_SM_DUW            : v = 64'(snap_q.smart.duw);
      SRC_SM_PC             : v = 64'(snap_q.smart.pc);
      SRC_SM_POH            : v = 64'(snap_q.smart.poh);
      SRC_SM_UNS            : v = 64'(snap_q.smart.uns);
      SRC_SM_ME             : v = 64'(snap_q.smart.me);
      SRC_SM_WT             : v = 64'(snap_q.smart.wt);
      SRC_SM_CT             : v = 64'(snap_q.smart.ct);
      SRC_SM_T1             : v = 64'(snap_q.smart.t1);
      SRC_V_VB              : v = 64'(snap_q.vrf.vb);
      SRC_V_VE              : v = 64'(snap_q.vrf.ve);
      SRC_V_OK              : v = 64'(snap_q.vrf.ok);
      SRC_V_BAD             : v = 64'(snap_q.vrf.bad);
      SRC_V_CRC             : v = 64'(snap_q.vrf.crc);
      SRC_V_PE              : v = 64'(snap_q.vrf.pe);
      SRC_V_OOO             : v = 64'(snap_q.vrf.ooo);
      SRC_V_PRT             : v = 64'(snap_q.vrf.prt);
      SRC_V_FBP             : v = 64'(snap_q.vrf.fbp);
      SRC_V_MVP             : v = 64'(snap_q.vrf.mvp);
      SRC_V_LSQ             : v = 64'(snap_q.vrf.lsq);
      SRC_V_LTS             : v = 64'(snap_q.vrf.lts);
      SRC_V_LST             : v = 64'(snap_q.vrf.lst);
      SRC_V_PAD             : v = 64'(snap_q.vrf.pad);
      SRC_V_FEP             : v = 64'(snap_q.vrf.fep);
      SRC_V_FEB             : v = 64'(snap_q.vrf.feb);
      SRC_V_EXP_DW          : v = 64'(snap_q.vrf.exp_dw);
      SRC_V_GOT_DW          : v = 64'(snap_q.vrf.got_dw);
      SRC_V_OVF             : v = 64'(snap_q.vrf.ovf);
      SRC_V_LFL             : v = 64'(snap_q.vrf.mvp_flags);   // WP8
      default: v = 64'd0;
    endcase
    return v;
  endfunction

  function automatic int tmpl_of(input logic [4:0] k, input logic [7:0] c);
    int t;
    t = -1;
    case (k)
      5'(MK_OK): begin
        case (c)
          8'(OP_PING):      t = T_OK_PING;
          8'(OP_CFGQ):      t = T_OK_CFG;
          8'(OP_TEL):       t = T_OK_TEL;
          8'(OP_SMART):     t = T_OK_SMART;
          8'(OP_SMARTQ):    t = T_OK_SMARTQ;
          8'(OP_MODE):      t = T_OK_MODE;
          8'(OP_RATE):      t = T_OK_RATE;
          8'(OP_LEN):       t = T_OK_LEN;
          8'(OP_SID):       t = T_OK_SID;
          8'(OP_DDRSZ):     t = T_OK_DDRSZ;
          8'(OP_PROF_CLR):  t = T_OK_PROF_PN;
          8'(OP_PROF_ADD),
          8'(OP_PROF_SET):  t = T_OK_PROF_I;
          8'(OP_PROF_REP):  t = T_OK_PROF_PR;
          8'(OP_PROF_END):  t = T_OK_PROF_PE;
          8'(OP_PROFQ):     t = T_OK_PROFQ;
          8'(OP_REC),
          8'(OP_REC_PROF):  t = T_OK_REC;
          8'(OP_STOP):      t = T_OK_STOP;
          8'(OP_PFAIL):     t = T_OK_PFAIL;
          8'(OP_ABORT):     t = T_OK_ABORT;
          8'(OP_READ_LAST),
          8'(OP_READ_N),
          8'(OP_READ_END):  t = T_OK_READ;
          8'(OP_READ_SID):  t = T_OK_READ_SID;
          8'(OP_FORMAT):    t = T_OK_FORMAT;
          8'(OP_MON):       t = T_OK_MON;
          8'(OP_MEMTEST):   t = T_OK_MEMTEST;
          8'(OP_DTEST):     t = T_OK_DTEST;
          8'(OP_DDRQ):      t = T_OK_DDR;
          default:          t = T_OK;
        endcase
      end
      5'(MK_ERR):       t = T_ERR;
      5'(MK_PROF_LINE): t = T_P_LINE;
      5'(MK_TEL):       t = T_TEL;
      5'(MK_CFGQ):      t = T_OK_CFG;
      5'(MK_SMARTQ):    t = T_OK_SMARTQ;
      5'(MK_DDRQ):      t = T_OK_DDR;
      5'(MK_EV_B):      t = T_EV_B;
      5'(MK_EV_S):      t = T_EV_S;
      5'(MK_EV_E):      t = T_EV_E;
      5'(MK_EV_RR):     t = T_EV_RR;
      5'(MK_EV_RP):     t = T_EV_RP;
      5'(MK_EV_RV):     t = T_EV_RV;
      5'(MK_EV_RF):     t = T_EV_RF;
      5'(MK_EV_RM):     t = T_EV_RM;
      5'(MK_EV_RD):     t = T_EV_RD;
      default:          t = -1;
    endcase
    return t;
  endfunction

  // ---------------------------------------------------------------- state
  typedef enum logic [3:0] {S_IDLE, S_FETCH, S_DEC, S_SP, S_TXT, S_EQ, S_VAL, S_NIB, S_END} st_e;
  st_e          st_q;
  logic [8:0]   pc_q;
  logic [95:0]  el_q;
  logic [79:0]  txt_q;
  logic         eq_q, fld_q, tick_pend_q;
  logic [63:0]  val_q;
  logic [4:0]   cnt_q;

  // two-stage source mux: stage 1 picks the low 4 bits of the index inside every group of 16
  localparam int NG = (NSRC + 15) / 16;
  logic [63:0] grp_d [NG];
  logic [63:0] grp_q [NG];
  always_comb begin
    for (int g = 0; g < NG; g++) grp_d[g] = src_val(g * 16 + int'(el_q[9:6]));
  end
  always_ff @(posedge clk) begin
    for (int g = 0; g < NG; g++) grp_q[g] <= grp_d[g];
  end

  // output FIFO 2048 x 8 -> uart_tx
  logic [7:0]  mem [0:2047];
  logic [10:0] wptr_q, rptr_q;
  logic [11:0] count_q;
  logic        put_v;
  logic [7:0]  put_d;
  logic [7:0]  rdata_q, tx_d_q;
  logic        rd_pend_q, tx_v_q, utx_ready;
  wire         afull = (count_q >= 12'd2040);
  wire         rd_issue = (count_q != 12'd0) && !rd_pend_q && !tx_v_q;

  function automatic logic [7:0] hexch(input logic [3:0] n);
    return (n < 4'd10) ? (8'h30 + 8'(n)) : (8'h37 + 8'(n));
  endfunction
  function automatic logic [3:0] nib_of(input logic [63:0] v, input logic [3:0] i);
    return v[i*4 +: 4];
  endfunction

  // ---------------------------------------------------------------- formatter engine
  always_ff @(posedge clk) begin
    put_v <= 1'b0;
    if (tel_tick) tick_pend_q <= 1'b1;
    if (rst) begin
      st_q        <= S_IDLE;
      msg_ready   <= 1'b0;
      tick_pend_q <= 1'b0;
      pc_q        <= '0;
      cnt_q       <= '0;
      eq_q        <= 1'b0;
      fld_q       <= 1'b0;
    end else begin
      case (st_q)
        S_IDLE: begin
          msg_ready <= 1'b1;
          if (msg_ready && msg_valid) begin
            if (tmpl_of(msg_kind, msg_code) >= 0) begin
              snap_q    <= snap;
              code_q    <= msg_code;
              arg0_q    <= msg_arg0;
              arg1_q    <= msg_arg1;
              pc_q      <= 9'(BASE[tmpl_of(msg_kind, msg_code)]);
              st_q      <= S_FETCH;
              msg_ready <= 1'b0;
            end
          end else if (msg_ready && tick_pend_q) begin
            snap_q      <= snap;
            code_q      <= 8'd0;
            arg0_q      <= 64'd0;
            arg1_q      <= 64'd0;
            pc_q        <= 9'(BASE[T_TEL]);
            st_q        <= S_FETCH;
            msg_ready   <= 1'b0;
            tick_pend_q <= tel_tick;
          end
        end
        S_FETCH: begin
          el_q <= ROM[pc_q];
          pc_q <= pc_q + 1'b1;
          st_q <= S_DEC;
        end
        S_DEC: begin
          case (el_q[95:94])
            K_TXT: begin txt_q <= el_q[93:14]; eq_q <= 1'b0; fld_q <= 1'b0; st_q <= S_TXT; end
            K_FLD: begin txt_q <= el_q[93:14]; eq_q <= (el_q[93:14] != 80'd0); fld_q <= 1'b1;
                         st_q <= S_SP; end
            default: st_q <= S_END;
          endcase
        end
        S_SP: begin
          if (!afull) begin put_v <= 1'b1; st_q <= S_TXT; end
        end
        S_TXT: begin
          if (txt_q == 80'd0) begin
            st_q <= eq_q ? S_EQ : (fld_q ? S_VAL : S_FETCH);
          end else if (txt_q[79:72] == 8'd0) begin
            txt_q <= {txt_q[71:0], 8'd0};
          end else if (!afull) begin
            put_v <= 1'b1;
            txt_q <= {txt_q[71:0], 8'd0};
          end
        end
        S_EQ: begin
          if (!afull) begin put_v <= 1'b1; st_q <= S_VAL; end
        end
        S_VAL: begin
          val_q <= grp_q[el_q[13:10]];
          cnt_q <= el_q[5:1];
          st_q  <= S_NIB;
        end
        S_NIB: begin
          if (!afull) begin
            put_v <= 1'b1;
            cnt_q <= cnt_q - 1'b1;
            if (cnt_q == 5'd1) st_q <= S_FETCH;
          end
        end
        S_END: begin
          if (!afull) begin
            put_v     <= 1'b1;
            st_q      <= S_IDLE;
            msg_ready <= 1'b1;
          end
        end
        default: st_q <= S_IDLE;
      endcase
    end
  end

  // character multiplexer for the FIFO write port (registered by put_v)
  always_ff @(posedge clk) begin
    case (st_q)
      S_SP:    put_d <= 8'h20;
      S_TXT:   put_d <= txt_q[79:72];
      S_EQ:    put_d <= 8'h3D;
      S_NIB:   put_d <= hexch(nib_of(val_q, 4'(cnt_q - 5'd1)));
      default: put_d <= 8'h0A;
    endcase
  end

  // ---------------------------------------------------------------- FIFO + UART
  always_ff @(posedge clk) begin
    if (put_v) mem[wptr_q] <= put_d;
    if (rd_issue) rdata_q <= mem[rptr_q];
  end
  always_ff @(posedge clk) begin
    if (rst) begin
      wptr_q    <= '0;
      rptr_q    <= '0;
      count_q   <= '0;
      rd_pend_q <= 1'b0;
      tx_v_q    <= 1'b0;
    end else begin
      if (put_v) wptr_q <= wptr_q + 1'b1;
      count_q <= count_q + {11'd0, put_v} - {11'd0, rd_issue};
      if (rd_issue) begin
        rptr_q    <= rptr_q + 1'b1;
        rd_pend_q <= 1'b1;
      end else if (rd_pend_q) begin
        tx_d_q    <= rdata_q;
        tx_v_q    <= 1'b1;
        rd_pend_q <= 1'b0;
      end
      if (tx_v_q && utx_ready) tx_v_q <= 1'b0;
    end
  end

  uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_uart_tx (
    .clk(clk), .rst(rst), .s_data(tx_d_q), .s_valid(tx_v_q), .s_ready(utx_ready), .txd(txd)
  );
endmodule
