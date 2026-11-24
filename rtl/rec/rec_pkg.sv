// =============================================================================
// rec_pkg.sv — types, codes and constants of the recorder (WP0).
//
// Domains : none (types only).  Spec: STAND/RECORDER_SPEC.md 1.2, 1.3, 2.3-2.6, 3, 4.4, 6.3.
// Rules   : does NOT import nvme_host_pkg (CODING_RULES.md 1).  All structs are packed;
//           the first member is the MSB.  Field order inside the CDC structs is only
//           a convention shared by producer and consumer (the CDC bridge moves them as
//           raw bit vectors).
// =============================================================================
package rec_pkg;

  // ---------------------------------------------------------------- constants
  localparam int unsigned UI_CLK_HZ   = 100_000_000;   // MIG ui_clk (4:1 of 400 MHz)
  localparam int unsigned LINK_CLK_HZ = 200_000_000;   // link_clk
  localparam int unsigned USER_CLK_HZ = 125_000_000;   // pcie user_clk
  localparam int          PROF_N      = 64;            // profile table depth
  localparam logic [7:0]  REC_VER     = 8'h01;         // `ver` of PING / $T (bumped by WP2 if needed)

  // data block (spec 3.1)
  localparam int          BLK_BYTES   = 4096;
  localparam int          BLK_BEATS   = 256;           // 16-byte beats per block
  localparam int          SLOT_BLOCKS = 32;            // 128 KiB slot
  localparam logic [31:0] BLK_MAGIC     = 32'h4352_564E;   // "NVRC"
  localparam logic [31:0] BLK_MAGIC_END = 32'h4244_4E45;   // "ENDB"
  localparam logic [7:0]  BLK_VER       = 8'd1;
  localparam int          FLG_FIRST = 0, FLG_LAST = 1, FLG_PAD = 2, FLG_TRUNC = 3,
                          FLG_PF = 4, FLG_DROP = 5, FLG_PROF = 6, FLG_BP = 7;   // block `flags` bits

  // CDC age limit seen by rec_ctrl (spec 4.3): 1 ms
  localparam int unsigned AGE_LIMIT_US = 1000;

  // ---------------------------------------------------------------- crc32_128
  // CRC-32 zlib (reflected, poly 0xEDB88320), one 128-bit beat per call.
  // Byte k of the beat is d[8k+7:8k], LSB first.  Raw register update:
  // the caller seeds with CRC32_INIT and inverts the final value (xorout).
  localparam logic [31:0] CRC32_INIT = 32'hFFFF_FFFF;

  function automatic logic [31:0] crc32_128(input logic [31:0] crc_in, input logic [127:0] d);
    logic [31:0] c;
    c = crc_in;
    for (int i = 0; i < 128; i++)
      c = (c[0] ^ d[i]) ? ((c >> 1) ^ 32'hEDB8_8320) : (c >> 1);
    return c;
  endfunction

  // ---------------------------------------------------------------- codes
  // Modes `md` (spec 1.2)
  typedef enum logic [3:0] {
    MD_NONE = 4'h0, MD_REC = 4'h1, MD_PF = 4'h2, MD_READ = 4'h3,
    MD_FORMAT = 4'h4, MD_MON = 4'h5, MD_DTEST = 4'h6, MD_MEMTEST = 4'h7
  } rec_mode_e;

  // rec_ctrl states `st` (spec 1.3)
  typedef enum logic [3:0] {
    ST_IDLE = 4'h0, ST_BRINGUP = 4'h1, ST_RUN = 4'h2, ST_SRC_STOP = 4'h3, ST_CORE_STOP = 4'h4,
    ST_RELEASE = 4'h5, ST_LOCAL = 4'h6, ST_ERROR = 4'h7, ST_CORE_RST = 4'h8
  } rec_state_e;

  // host_sequencer states `cs` (as reported by the core through core_stat_t.seq_state)
  localparam logic [3:0] CS_IDLE = 4'h0, CS_RUN = 4'h5, CS_DRAIN = 4'h6, CS_FLUSH = 4'h7,
                         CS_SHUTDOWN = 4'h8, CS_DONE = 4'h9, CS_ERROR = 4'hF;

  // Command opcodes of rec_cmd_parser (spec 2.3).  OP_PERR = parse error, cmd_err holds the code.
  typedef enum logic [5:0] {
    OP_NOP = 6'd0,  OP_PING = 6'd1,  OP_STAT = 6'd2,  OP_CFGQ = 6'd3,  OP_RESQ = 6'd4,
    OP_TEL = 6'd5,  OP_SMART = 6'd6, OP_SMARTQ = 6'd7, OP_MODE = 6'd8, OP_RATE = 6'd9,
    OP_LEN = 6'd10, OP_SID = 6'd11,  OP_DDRSZ = 6'd12,
    OP_PROF_CLR = 6'd13, OP_PROF_ADD = 6'd14, OP_PROF_SET = 6'd15, OP_PROF_REP = 6'd16,
    OP_PROF_END = 6'd17, OP_PROFQ = 6'd18,
    OP_REC = 6'd19, OP_REC_PROF = 6'd20, OP_STOP = 6'd21, OP_PFAIL = 6'd22,
    OP_READ_LAST = 6'd23, OP_READ_N = 6'd24, OP_READ_END = 6'd25, OP_READ_SID = 6'd26,
    OP_FORMAT = 6'd27, OP_MON = 6'd28, OP_MEMTEST = 6'd29, OP_DTEST = 6'd30,
    OP_DDRQ = 6'd31, OP_ABORT = 6'd32,
    OP_PERR = 6'd63
  } rec_op_e;

  // ERR codes (spec 2.4)
  typedef enum logic [7:0] {
    ERR_SYNTAX = 8'h01, ERR_UNKNOWN = 8'h02, ERR_RANGE = 8'h03, ERR_STATE = 8'h04,
    ERR_NOCAL = 8'h05, ERR_CORE = 8'h06, ERR_PROF_EMPTY = 8'h08, ERR_PROF_FULL = 8'h09,
    ERR_LONG = 8'h0A, ERR_NOREC = 8'h0B, ERR_BUSY = 8'h0C, ERR_UART = 8'h0D
  } rec_err_e;

  // Message kinds rec_ctrl -> rec_tx_fmt (msg_kind[4:0]).  msg_code = opcode (OK), error code (ERR)
  // or sub-code of the event.  Proposed by WP0 for WP1/WP2 (spec leaves the encoding open).
  typedef enum logic [4:0] {
    MK_NONE = 5'd0,  MK_OK = 5'd1,   MK_ERR = 5'd2,   MK_PROF_LINE = 5'd3,
    MK_EV_B = 5'd4,  MK_EV_S = 5'd5,  MK_EV_E = 5'd6,
    MK_EV_RR = 5'd7, MK_EV_RP = 5'd8, MK_EV_RV = 5'd9, MK_EV_RF = 5'd10,
    MK_EV_RM = 5'd11, MK_EV_RD = 5'd12, MK_TEL = 5'd13, MK_SMARTQ = 5'd14,
    MK_CFGQ = 5'd15, MK_DDRQ = 5'd16
  } rec_msg_kind_e;

  // ---------------------------------------------------------------- structs
  // SMART / Health log (spec 6.3): 384 bits, bit layout of nvme_admin.log_smart.
  typedef struct packed {
    logic [15:0] t1;        // [383:368] Temperature Sensor 1   <- bytes 200-201
    logic [31:0] ct;        // [367:336] Critical Temp Time     <- 196-199
    logic [31:0] wt;        // [335:304] Warning Temp Time      <- 192-195
    logic [31:0] me;        // [303:272] Media Errors           <- 160-163
    logic [31:0] uns;       // [271:240] Unsafe Shutdowns       <- 144-147
    logic [31:0] poh;       // [239:208] Power On Hours         <- 128-131
    logic [31:0] pc;        // [207:176] Power Cycles           <- 112-115
    logic [63:0] duw;       // [175:112] Data Units Written     <- 48-55
    logic [63:0] dur;       // [111:48]  Data Units Read        <- 32-39
    logic [7:0]  rsvd;      // [47:40]   0
    logic [7:0]  sp_used;   // [39:32]   Percentage Used        <- 5
    logic [7:0]  sp;        // [31:24]   Available Spare        <- 3
    logic [15:0] tk;        // [23:8]    Composite Temp (K)     <- 1-2
    logic [7:0]  cw;        // [7:0]     Critical Warning       <- 0
  } smart_t;

  // XADC raw codes (12 bit), spec 4.4 WP7
  typedef struct packed {
    logic [11:0] temp;
    logic [11:0] vccint;
    logic [11:0] vccaux;
    logic [11:0] vccbram;
    logic        valid;
  } xadc_t;

  // Session configuration link -> ui, X2 (only changes in IDLE).  `lbads` is added by WP0:
  // rec_verify needs it, the spec says "through rec_cfg_t" (3.3 / 4.4 WP6).
  typedef struct packed {
    logic        bp;          // 1 = BP mode, 0 = RT
    logic [31:0] sid;
    logic [31:0] rsid;
    logic [10:0] ddr_mib;     // ddr_fifo size_mib
    logic [6:0]  prof_n;      // nseg 1..64
    logic [15:0] prof_rep;
    logic        prof_end;    // 0 HOLD, 1 STOP
    logic [10:0] mt_mib;
    logic [3:0]  lbads;       // from core_stat, quasi-static
  } rec_cfg_t;

  // rec_verify totals (spec 3.3, !RV fields of 2.6)
  typedef struct packed {
    logic [47:0] vb;          // verified bytes
    logic [31:0] ve;          // erroneous beats
    logic [39:0] ok;          // OK blocks
    logic [31:0] bad;
    logic [31:0] crc;
    logic [31:0] pe;          // payload beats with error
    logic [31:0] ooo;
    logic [31:0] prt;         // partial beats
    logic [39:0] fbp;         // first bad position (all ones = none)
    logic [39:0] mvp;         // max valid position
    logic [63:0] lsq;         // seq of mvp block
    logic [63:0] lts;         // ts of mvp block
    logic [31:0] lst;         // lost of mvp block
    logic [31:0] pad;
    logic [39:0] fep;         // first payload error: beat position
    logic [7:0]  feb;         // first payload error: byte/DW index
    logic [31:0] exp_dw;      // expected DW
    logic [31:0] got_dw;      // received DW
    logic        ovf;         // tap FIFO overflow: result invalid
    logic [7:0]  mvp_flags;   // WP8: block flags of the max-valid block (bit 1 = LAST); !RV lfl (audit D17 / WP6 proposal)
  } vrf_res_t;

  // ui_clk statistics -> link, X4
  typedef struct packed {
    // source (rec_framer / rec_profile)
    logic [47:0] src_blocks;
    logic [47:0] wr_blocks;
    logic [31:0] drop_blocks;
    logic [15:0] pad_blocks;
    logic [15:0] trunc_blocks;
    logic [39:0] bp_max_beats;
    logic [15:0] rt_stall_max;
    logic [31:0] step;
    logic [5:0]  seg_idx;
    logic [15:0] rep_cnt;
    logic        prof_active;
    logic        src_active;
    logic        src_done;       // framer.done (level)
    logic        len_reached_s;  // sticky until ui_clear (pulse of the framer made level)
    logic        prof_end_s;     // sticky until ui_clear
    // DDR
    logic [30:0] fill_bytes;
    logic [30:0] fill_max_bytes;
    logic        ddr_idle;
    logic [47:0] ddr_wr_bytes;
    logic [47:0] ddr_rd_bytes;
    logic        calib;          // init_calib_complete
    logic        ui_ok;          // ui_rst released
    // MEMTEST
    logic        mt_busy;
    logic        mt_done;
    logic [31:0] mt_err_cnt;
    logic [28:0] mt_first_err_addr;
    // DTEST (rec_sink_chk)
    logic [47:0] dt_beats;
    logic [31:0] dt_hdr_err;
    // verify
    vrf_res_t    vrf;
    logic        end_found;
  } ui_stat_t;

  // user_clk (core) statistics -> link, X10
  typedef struct packed {
    logic [3:0]  seq_state;      // host_sequencer state (CS_*)
    logic [7:0]  error;          // status_error
    logic [7:0]  error_detail;   // status_error_detail
    logic [7:0]  flags;          // status_flags
    logic        link_up;
    logic        gen2;
    logic [1:0]  link_width;     // as status_link_width
    logic        lba_range_err;
    logic [63:0] bytes_done;     // stat_bytes_done
    logic [63:0] cmds_done;
    logic [31:0] cmd_errors;
    logic [31:0] max_latency;
    logic [63:0] beats_in;       // stat_beats_in
    logic [3:0]  lbads;
    logic [63:0] nsze;
    smart_t      smart;          // log_smart
    logic [15:0] log_cnt;
    logic [15:0] log_err_cnt;
    logic        log_busy;
    logic [15:0] log_status;
    logic [31:0] rtd3e_us;
    logic        cmd_limit_done;
  } core_stat_t;

  // Snapshot taken by rec_tx_fmt when it accepts a message (spec 4.4 WP1/WP2).  Superset of the
  // $T fields (2.5) and of the value fields needed by the CFG?/SMART?/DDR?/!R* lines (2.3, 2.6, 7.2).
  typedef struct packed {
    // $T
    logic [31:0] t_ms;
    logic [3:0]  st;
    logic [3:0]  md;
    logic        cok;
    logic        uok;
    logic [3:0]  cs;
    logic [7:0]  ce;
    logic [7:0]  cd;
    logic [7:0]  fl;
    logic [3:0]  lk;
    logic        cal;
    logic [31:0] stp;
    logic [7:0]  seg;
    logic [15:0] rep;
    logic [47:0] src;
    logic [47:0] wr;
    logic [31:0] drp;
    logic [31:0] ddr;
    logic [31:0] ddrm;
    logic [39:0] bpm;
    logic [31:0] rng;
    logic [63:0] ssd;
    logic [31:0] cmd;
    logic [31:0] cer;
    logic [31:0] lat;
    logic [47:0] vb;
    logic [31:0] ve;
    logic [15:0] tk;
    logic [7:0]  cw;
    logic [15:0] smo;
    logic [15:0] sme;
    logic [11:0] xt;
    logic [11:0] xvi;
    logic [11:0] xva;
    logic [11:0] xvb;
    // CFG?
    logic        cfg_bp;
    logic [31:0] cfg_stp;
    logic [31:0] cfg_len;
    logic [31:0] cfg_sid;
    logic [15:0] cfg_ddr;
    logic [15:0] cfg_tel;
    logic [15:0] cfg_sm;
    logic [7:0]  cfg_pn;
    logic [15:0] cfg_pr;
    logic        cfg_pe;
    logic [31:0] cfg_rsid;
    // SMART? / !RV / DDR? / !R* extras
    smart_t      smart;
    vrf_res_t    vrf;
    logic        end_found;
    logic [31:0] mte;            // memtest errors
    logic [31:0] mta;            // memtest first error address
    logic [31:0] mt_bytes;       // !RM byt
    logic [31:0] dt_he;          // !RD he
    logic [47:0] dt_wr;          // !RD wr
    logic [15:0] pad_blocks;
    logic [15:0] trunc_blocks;
    logic [63:0] dur;            // link_clk ticks of the session
    logic [31:0] sid;
    logic [31:0] rtd;      // RTD3E, us (full 32 bit, no saturation)
    // !RP (7.2)
    logic        rp_src;
    logic        rp_inb;
    logic        rp_lat;
    logic        rp_lov;
    logic [47:0] rp_pnd;
    logic [31:0] rp_pdd;
    logic [31:0] rp_prg;
    logic [31:0] rp_pad;
    logic [63:0] rp_t1;
    logic [63:0] rp_t2;
    logic [63:0] rp_t3;
    logic [63:0] rp_t4;
    logic [63:0] rp_t5;
    logic [63:0] rp_t6;
    logic        rp_ok;
    logic [39:0] rp_lpos;
  } rec_snap_t;

endpackage
