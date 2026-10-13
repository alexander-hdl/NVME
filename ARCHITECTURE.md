# NVMe host on bare Kintex-7 — architecture specification

Status: v1.0 (design contract for implementation). Owner: project architect.
Language: SystemVerilog-2012 (synthesizable subset accepted by Vivado 2022.2 and xsim).
Target: Xilinx Kintex-7 (XC7K325T-2 reference), `pcie_7x` v3.3 integrated block, **Root Port, Gen2 x4,
AXI-Stream 128-bit @ 125 MHz** (`user_clk`). Goal: sustained sequential NVMe write at the PCIe Gen2 x4
ceiling (≈1.7–1.8 GB/s payload), with no CPU of any kind.

Research inputs: `research/01_open_source_survey.md`, `research/02_nvme_minimal_host_brief.md`,
`research/03_pcie_7x_root_port_brief.md`. Every numbered decision below cites them where relevant.

---

## 1. Decisions and rationale

| # | Decision | Why |
|---|---|---|
| D1 | Raw TLP interface of `pcie_7x` (not `axi_pcie`) | `axi_pcie` caps MPS at 256 B and allows only 4 outstanding upstream MRd; raw TLP lets the completer stream CplD at line rate with deep tag queueing (research/03 §12). |
| D2 | Gen2 **x4**, 128-bit @ 125 MHz | NVMe M.2 SSDs are x4; 128b@125 closes timing easily on -2; beat efficiency for 256 B CplD equals the 64b@250 option (research/02 §7). Straddling on the 128-bit RX is confined to one adapter module. |
| D3 | **Internal TLP stream format** (§3) is clean: one TLP per packet, header in beat 0, payload from beat 1 DW0, little-endian bytes | Isolates every 7-series quirk (straddle, 3DW misalignment, big-endian payload DWs) in two adapter modules; all NVMe logic sees "memory order". |
| D4 | Host memory = on-chip BRAM; **no DDR** in the base design | "Bare FPGA" generality; completer latency is minimal (no AXI round trip); DDR ring is a documented extension. |
| D5 | Data ring of `N_SLOTS` × `SLOT_BYTES` (default 8 × 128 KiB = 1 MiB) | 128 KiB commands fit every MDTS ≥ 5; QD up to 8 covers ≈500 µs of SSD latency at 1.8 GB/s (research/02 §8). |
| D6 | **PRP lists are synthesized arithmetically**, not stored | `entry[i] = slot_base + (i+1)·4096` is a pure function (mcrl "inverted queue", research/01 §1.3). Saves RAM and teaches the key insight. |
| D7 | SQs are **real BRAM** written by host logic; CQs are real BRAM written by the SSD and polled by host logic | Mirrors the software model exactly (teaching value); SQE writes are rare (one per 128 KiB). |
| D8 | One I/O SQ/CQ pair (QID 1), 64 entries each; admin queues 32 entries | More queues give nothing for a single sequential stream. |
| D9 | MPS = min(SSD MPSS, `MPS_CAP`=256 by default); MRRS = 512 B; Extended Tags enabled if ETFS | 256 B is universal; 512 B MRRS keeps the completer's split logic simple and the NP queue short (research/02 §6). |
| D10 | Polling CQs, interrupts disabled (Command.INTxDis, IEN=0) | No CPU; MSI MWr if ever sent is harmlessly dropped. |
| D11 | All host addresses < 4 GB → the SSD uses 3DW headers | Simplifies RX/TX adapters (research/02 pitfall 13). |
| D12 | FPGA drives PERST#; ≥100 ms hold, ≥100 ms before first CfgRd, CRS retry ≤ 1 s | PCIe CEM rules, pl-nvme experience (research/01 pitfall 7). |
| D13 | Root Port's own Command.BME and DevCtl.MPS programmed via `cfg_mgmt` | Without BME every upstream MRd gets UR (PG054 Table 3-25; research/03 §4). |
| D14 | Orderly shutdown on stop: Flush → Delete SQ/CQ → CC.SHN → wait SHST=10b | Consumer SSDs have no power-loss protection (research/02 §9). |
| D15 | Verification with xsim: behavioural SSD model attached at the **core-facing AXI-Stream** (including straddle/misalignment rules) | Exercises adapters; no GT simulation needed. |

---

## 2. Memory map (addresses as seen by the SSD, 32-bit)

| Region | Base | Size | Backing | Who reads / writes over PCIe |
|---|---|---|---|---|
| Admin SQ (ASQ) | `0x1000_0000` | 32 × 64 B = 2 KiB | `asq_ram` | SSD MRd |
| Admin CQ (ACQ) | `0x1000_1000` | 32 × 16 B = 512 B | `acq_ram` | SSD MWr |
| Identify buffer | `0x1000_2000` | 4 KiB | `ident_ram` | SSD MWr |
| I/O SQ (IOSQ, QID 1) | `0x1000_3000` | 64 × 64 B = 4 KiB | `iosq_ram` | SSD MRd |
| I/O CQ (IOCQ, QID 1) | `0x1000_4000` | 64 × 16 B = 1 KiB | `iocq_ram` | SSD MWr |
| PRP lists | `0x2000_0000 + k·0x1000` | one page per slot k | synthesized | SSD MRd |
| Data ring | `0x4000_0000 + k·SLOT_BYTES` | `N_SLOTS` × `SLOT_BYTES` | `data_ring` BRAM | SSD MRd |
| SSD BAR0 | `0xC000_0000` | ≥16 KiB (sized at enumeration) | inside the SSD | host MRd/MWr (registers, doorbells) |

Anything else → `Cpl` with status UR (reads) / silently dropped (writes).
All constants live in `rtl/nvme_host_pkg.sv`.

PRP synthesis for a read of address `A` in the PRP window: `k = A[31:12] - 0x20000`, entry index `i = A[11:3]`,
value = `DATA_BASE + k·SLOT_BYTES + (i+1)·4096` as a 64-bit little-endian quantity (upper DW = 0).
Entries ≥ `SLOT_BYTES/4096 − 1` are never fetched by a correct SSD; return zeros.

---

## 3. Internal TLP stream ("tlp_axis")

AXI4-Stream, `DATA_W = 128`, `user_clk` domain, `tvalid/tready/tlast/tkeep[15:0]`, plus `tuser`:

| Beat | `tdata[31:0]` | `[63:32]` | `[95:64]` | `[127:96]` |
|---|---|---|---|---|
| 0 (header) | H0 (Fmt/Type/TC/TD/EP/Attr/Length) | H1 (ReqID/Tag/BE or CplID/Status/BC) | H2 (Addr or ReqID/Tag/LowerAddr) | H3 (Addr[63:32] for 4DW, else 0) |
| 1..n (payload) | D0 / D4 / ... | D1 / D5 | D2 / D6 | D3 / D7 |

Rules:
- Header DW field layout is exactly the PCIe spec layout (bit 31 of H0 is the MSB of byte 0 of the TLP). This matches the 7-series core, no swap.
- **Payload is in memory order**: `tdata[7:0]` of a payload DW is the byte at the lowest address. (The 7-series core presents payload DWs big-endian; the adapters swap.) Consequence: a 128-bit payload beat equals 16 consecutive bytes of host memory, byte 0 in `[7:0]`.
- One TLP per packet, header always in a beat of its own, payload always starts at beat 1 DW0. No straddling.
- `tkeep` is DW-granular (`4'hF` per DW); on the last beat unused upper DWs have `tkeep=0`; on all other beats `tkeep=16'hFFFF`. Header-only TLPs are one beat with `tkeep=16'hFFFF`, `tlast=1`.
- `tuser[0] = err_fwd` (poisoned, RX only), `tuser[1] = ecrc_err` (RX only). TX side `tuser` unused (0).
- `tready` may be deasserted at any time by the sink; the source must hold data stable.

Helper functions in the package: `tlp_fmt_type(h0)`, `tlp_len_dw(h0)`, `is_mrd/is_mwr/is_cpl/is_cpld(h0)`, `hdr_mrd32(addr,len,reqid,tag,fbe,lbe)`, `hdr_mwr32(...)`, `hdr_cfgrd0/cfgwr0(bdf, reg, reqid, tag, fbe)`, `hdr_cpld(cplid, status, bc, reqid, tag, lower_addr, len)`, `hdr_cpl(...)`, `bswap32(x)`.

---

## 4. Module hierarchy

```
nvme_host_top                       (board-level: refclk, GT lanes, PERST#, user data stream, control/status)
├── pcie7x_wrapper                  (instantiates generated IP pcie_7x_0; exposes core AXI-S, cfg_mgmt, pl_*, user_clk/rst)
└── nvme_host_core                  (everything below is pure RTL in user_clk domain)
    ├── pcie7x_rx_adapt             core m_axis_rx (128b, straddle, misaligned 3DW) -> tlp_axis
    ├── pcie7x_tx_adapt             tlp_axis -> core s_axis_tx (contiguous DWs, byte-swapped payload)
    ├── tlp_rx_demux                by Fmt/Type: Cpl* -> requester, MRd -> completer, MWr -> writer, rest dropped
    ├── tlp_tx_arb                  2 inputs (completer high priority, requester) -> one tlp_axis
    ├── hostmem_completer           MRd -> CplD stream (split at MPS, BC/LA, UR for unknown addr); reads from mem sources
    ├── hostmem_writer              MWr -> ident_ram / acq_ram / iocq_ram (byte-enable writes, DW realign); drops others
    ├── hostmem_rams                asq_ram, iosq_ram, acq_ram, iocq_ram, ident_ram (dual-port BRAM, 128-bit wide)
    ├── prp_synth                   PRP window read generator (pure function)
    ├── data_ring                   input AXI-S (async) -> BRAM ring of N_SLOTS; slot_full/slot_free; 128-bit read port
    ├── rp_requester                CfgRd0/CfgWr0/MRd32/MWr32 issue + completion match + timeout + CRS reporting
    ├── req_mux                     2 command ports (admin FSM; write engine doorbells) -> rp_requester
    ├── link_bringup                PERST# sequencing, link-up/Gen2 wait, 100 ms timers
    ├── rp_cfg_init                 cfg_mgmt writes: Command.BME, DevCtl.MPS/ExtTag, bus numbers
    ├── pcie_enum                   SSD config space: VID/DID, BAR0 sizing/assignment, cap walk, DevCtl MPS/MRRS, Command
    ├── nvme_admin                  CAP/CC/CSTS, AQA/ASQ/ACQ, enable, Identify ×2, Set Features, Create CQ/SQ, shutdown
    ├── nvme_write_engine           slot -> SQE in iosq_ram, SQ doorbell, CQ reaping (phase), slot release, Flush, stats
    └── host_sequencer              top-level control FSM: link -> enum -> rp_cfg -> admin init -> run -> stop -> shutdown; error codes
```

### 4b. Common primitives (`rtl/common/`) — exact interfaces (contract)

```systemverilog
// Register slice, full throughput, breaks the tready combinational path.
module axis_skid #(parameter W = 128, parameter UW = 2) (
  input clk, input rst,
  input  [W-1:0] s_tdata, input [W/8-1:0] s_tkeep, input s_tlast, input [UW-1:0] s_tuser, input s_tvalid, output s_tready,
  output [W-1:0] m_tdata, output [W/8-1:0] m_tkeep, output m_tlast, output [UW-1:0] m_tuser, output m_tvalid, input m_tready);

// Synchronous FIFO for AXI-S beats (stores tdata/tkeep/tlast/tuser). DEPTH power of 2. First-word fall-through.
module axis_fifo_sync #(parameter W = 128, parameter UW = 2, parameter DEPTH = 16) (
  input clk, input rst,
  input  [W-1:0] s_tdata, input [W/8-1:0] s_tkeep, input s_tlast, input [UW-1:0] s_tuser, input s_tvalid, output s_tready,
  output [W-1:0] m_tdata, output [W/8-1:0] m_tkeep, output m_tlast, output [UW-1:0] m_tuser, output m_tvalid, input m_tready,
  output [$clog2(DEPTH):0] count, output almost_full /* count >= DEPTH-2 */);

// Asynchronous FIFO (gray pointers, 2-FF synchronisers), data only. DEPTH power of 2.
module axis_fifo_async #(parameter W = 128, parameter DEPTH = 512) (
  input s_clk, input s_rst, input [W-1:0] s_tdata, input s_tvalid, output s_tready,
  input m_clk, input m_rst, output [W-1:0] m_tdata, output m_tvalid, input m_tready);

// Simple dual-port RAM, byte-enable write, 2-cycle read latency (BRAM output register + pipeline register).
module sdp_ram #(parameter DEPTH = 256, parameter W = 128) (
  input clk,
  input wr_en, input [W/8-1:0] wr_be, input [$clog2(DEPTH)-1:0] wr_addr, input [W-1:0] wr_data,
  input rd_en, input [$clog2(DEPTH)-1:0] rd_addr, output [W-1:0] rd_data /* valid 2 cycles after rd_en */);

// DW shifter for payload-only packet streams (tkeep is DW-granular: 4'hF per present DW).
// dir=0 (left): drop the first `shift` DWs of the packet and compact; the packet may end one beat earlier.
// dir=1 (right): insert `shift` empty DWs (tkeep=0) before DW0; the packet may need one extra beat.
// `shift`/`dir` are sampled together with the first beat of each packet. Full rate: one input beat per cycle.
module dw_shift (
  input clk, input rst,
  input  [127:0] s_tdata, input [15:0] s_tkeep, input s_tlast, input s_tvalid, output s_tready,
  input  [1:0] s_shift, input s_dir,
  output [127:0] m_tdata, output [15:0] m_tkeep, output m_tlast, output m_tvalid, input m_tready);

// Millisecond tick generator: `tick_ms` pulses once per ms (or once per µs when SIM_FAST=1).
module timer_ms #(parameter CLK_HZ = 125_000_000, parameter SIM_FAST = 0) (input clk, input rst, output tick_ms);
```

Usage: `pcie7x_tx_adapt` = `dw_shift(right, 3)` on the payload, merging the first shifted beat with `{H2,H1,H0}`
for 3DW headers. `hostmem_completer` = memory beats → `dw_shift(left, addr[3:2])` → prepend header beat.
`hostmem_writer` = payload → `dw_shift(right, addr[3:2])` → byte-enable write beats.

Clocking: one domain `user_clk` (125 MHz) for everything except the user data input (`data_clk`, any frequency,
crossed inside `data_ring` by an async FIFO) and the free-running 100 MHz-derived `init_clk` used only by `link_bringup`
timers before `user_clk` is stable (the pcie_7x `user_clk_out` is present once the MMCM locks; `link_bringup` runs on
`sys_clk`-derived clock to be safe).

Reset: `user_reset` from the core (synchronous, active high) → all `nvme_host_core` logic. `link_bringup` has its own
reset from the board.

---

## 5. Module interface specifications

Signal naming: `s_` = slave/sink, `m_` = master/source. All streams are tlp_axis (§3) unless stated.
All modules: `input logic clk, input logic rst` (rst synchronous, active high).

### 5.1 `pcie7x_rx_adapt`
```
// core side (PG054 128-bit RX)
input  [127:0] m_axis_rx_tdata;  input [15:0] m_axis_rx_tkeep;  input m_axis_rx_tlast;  // tlast unused in 128b mode
input          m_axis_rx_tvalid; output       m_axis_rx_tready; input [21:0] m_axis_rx_tuser;
output         rx_np_ok;         // 1 unless the internal MRd queue is nearly full (see completer)
input          np_backpressure;  // from completer: request FIFO almost full
// internal
output tlp_axis m_tlp (tdata, tkeep, tlast, tvalid, tuser[1:0]); input m_tlp_tready;
```
Behaviour: decode `is_sof = tuser[14]`, `sof_at_dw2 = tuser[13]`, `is_eof = tuser[21]`, `eof_dw = tuser[20:19]`.
Handle: SOF at DW0 or DW2; EOF at any DW; straddle (EOF of pkt A at DW0/DW1 and SOF of pkt B at DW2 in the same beat);
3DW header → payload starts at DW3 of the SOF beat (or DW1 of the next beat when SOF at DW2); 4DW header → see
research/03 §2 table. Output: header in beat 0 (H3 = 0 for 3DW), payload re-packed from beat 1 DW0, payload DWs
byte-swapped (`bswap32`). `tuser[0] = tuser_core[1]` (err_fwd), `tuser[1] = tuser_core[0]` (ecrc_err at EOF).
Length: for data TLPs use `Length` from H0 to compute the last beat's `tkeep`; do not rely on core `tkeep`.
Throughput: must accept one core beat per cycle when the sink is ready, except that a straddled beat may take 2 output
beats (internal elastic buffer ≥ 4 beats). `m_axis_rx_tready` may be deasserted (core allows it).
`rx_np_ok = ~np_backpressure`.

### 5.2 `pcie7x_tx_adapt`
```
input  tlp_axis s_tlp; output s_tlp_tready;
output [127:0] s_axis_tx_tdata; output [15:0] s_axis_tx_tkeep; output s_axis_tx_tlast; output s_axis_tx_tvalid;
input  s_axis_tx_tready; output [3:0] s_axis_tx_tuser;   // = 4'b0000 (no streaming mode; store-and-forward is safe)
input  [5:0] tx_buf_av;  // informational; tie through to status
```
Behaviour: if H0 is 3DW (Fmt[1]=0 i.e. bit 29 == 0), drop H3 and shift the payload left by one DW so the core sees
`{D0,H2,H1,H0}` then `{D4,D3,D2,D1}`...; if 4DW, pass unchanged. Byte-swap payload DWs. Generate `tkeep` per PG054
(`0x000F/0x00FF/0x0FFF/0xFFFF` on the last beat, all-ones otherwise) and `tlast`. **Must sustain one beat per cycle**
with no bubbles while `s_axis_tx_tready=1` (completer data path). Hold data stable when `tready=0`.
Never deassert `tvalid` mid-packet (the core requires it for `tx_str`; we keep `tx_str=0` but still honour it).

### 5.3 `tlp_rx_demux`
```
input tlp_axis s_tlp; output s_tlp_tready;
output tlp_axis m_cpl, m_mrd, m_mwr;  (each with tready input)
output logic drop_cnt_inc;  // pulse per dropped TLP (Msg, Cfg type1, unknown, poisoned MWr)
```
Route on beat 0 (Fmt/Type of H0): `Cpl/CplD` → `m_cpl`; `MRd32/MRd64` → `m_mrd`; `MWr32/MWr64` → `m_mwr` (if
`tuser[0]`=poison → drop whole packet, count); anything else → drop whole packet (consume all beats), count.
Packet-level routing: decision latched at beat 0 for the rest of the packet.

### 5.4 `tlp_tx_arb`
```
input tlp_axis s_cpl (from completer), s_req (from requester); output ready for each;
output tlp_axis m_tlp; input m_tlp_tready;
```
Packet-granular arbitration. Priority: completer, but if both have a packet pending at a boundary and the completer
won the previous round, let the requester go once (prevents doorbell starvation). Zero-bubble passthrough (use
`axis_skid` on the output if needed to keep full rate).

### 5.5 `hostmem_completer`
```
input tlp_axis s_mrd; output s_mrd_tready;         // MRd headers only (header-only packets)
output np_backpressure;                           // request FIFO has < 2 free entries
output tlp_axis m_cpl; input m_cpl_tready;         // CplD / Cpl(UR)
input  [2:0] mps_code;                             // 0=128,1=256,2=512 (programmed value)
input  [15:0] completer_id;                        // 16'h0000 (RP bus 0 dev 0 fn 0)
// memory read ports (all 128-bit, 2-cycle latency, 16-byte aligned beat addresses)
output [ADDR_W-1:0] rd_addr; output rd_en; output [1:0] rd_sel;  // sel: 0=asq,1=iosq,2=prp,3=data
input  [127:0] rd_data; input rd_valid;            // rd_valid = rd_en delayed 2 (sources are always ready)
```
Behaviour:
1. Pop MRd headers into a request FIFO (depth 16 ≥ 12 NP credits of the core).
2. For each request: decode `addr` (H2 for 3DW; if 4DW with H2≠0 → UR) into region (§2). Unknown → one `Cpl` with
   Status=UR (`BC` = requested bytes, `LA` = addr[6:0]), Length=0 (header-only packet). Poisoned/ECRC flags are
   not applicable (requests have no payload).
3. For known regions: total bytes = `len_dw·4` (len 0 → 1024 DW = 4096 B; MRd length ≤ MRRS=512 B expected but
   support up to 4096). Split into CplDs: first CplD runs from `addr` to the next `MPS`-aligned boundary (or end);
   subsequent CplDs are MPS-sized, MPS-aligned; last may be shorter. For each CplD: `Length` = DWs in this CplD,
   `BC` = bytes remaining including this CplD (first BC adjusted by First/Last BE per PCIe Table 2-27 for the
   1-DW and multi-DW cases; for BE=F/F it is simply bytes remaining), `LA` = low 7 bits of this CplD's start byte
   address (for the first CplD: address of the first enabled byte). Status SC, CplID=`completer_id`, ReqID/Tag/TC/Attr
   copied from the request. 0-length read (len=1, FBE=0): one 1-DW CplD, BC=1.
4. Data path: issue `rd_en` for each 16-byte beat covering the CplD (start address rounded down to 16 B), stream the
   returned beats through a `dw_realign` by `addr[3:2]` so payload begins at beat 1 DW0 of the output packet; compute
   output `tkeep` from Length. **Pipeline so that consecutive CplDs (and consecutive requests) flow without bubbles**;
   use prefetch so BRAM latency is hidden (issue reads ahead; an output FIFO of ≥ 8 beats absorbs `m_cpl_tready`
   stalls; stop issuing reads when the FIFO has < 4 free beats).
5. Completions of one request go out in increasing address order; requests are served in FIFO order (simplest and
   legal; per-tag ordering is preserved automatically).

### 5.6 `hostmem_writer`
```
input tlp_axis s_mwr; output s_mwr_tready;
// write ports (128-bit, byte enables), one per target
output [11:0] ident_wr_addr /*16B beats*/; output [127:0] wr_data; output [15:0] wr_be; output ident_wr_en, acq_wr_en, iocq_wr_en;
output [ADDR_W-1:0] wr_beat_addr;   // shared beat address, region-relative
output drop_cnt_inc;                // MWr to unmapped address (e.g. MSI) or poisoned
```
Behaviour: decode H2 (3DW; 4DW with H3≠0 → drop). Compute first-beat byte enables from `addr[3:2]` and First BE,
last from Last BE and Length; use `dw_realign` to shift payload DWs **right** by `addr[3:2]` so each output beat
is a 16-byte aligned memory beat with the proper `wr_be`. A MWr may cover several beats; MWr never crosses 4 KiB
(core-checked). Regions: Identify buffer, ACQ, IOCQ; any other → consume and count.

### 5.7 `hostmem_rams`
Five `sdp_ram` instances, all 128-bit wide with byte enables on the write port:

| RAM | Depth (16 B beats) | Write port owner | Read port owner |
|---|---|---|---|
| `asq_ram` | 128 (2 KiB) | `nvme_admin` (SQE writes, 4 beats per SQE) | completer (`rd_sel=0`) |
| `iosq_ram` | 256 (4 KiB) | `nvme_write_engine` | completer (`rd_sel=1`) |
| `acq_ram` | 32 (512 B) | `hostmem_writer` (+ clear FSM at init) | `nvme_admin` CQ poll |
| `iocq_ram` | 64 (1 KiB) | `hostmem_writer` (+ clear) | `nvme_write_engine` CQ poll |
| `ident_ram` | 256 (4 KiB) | `hostmem_writer` | `nvme_admin` field extraction |

A 2-cycle read latency is the contract everywhere. The completer's `rd_sel` mux sits in `nvme_host_core`.
CQ RAMs expose a `clear` input (sequential zero fill by a small counter; `clear_done` output) used before CC.EN.

### 5.8 `prp_synth`
Combinational-with-register source for `rd_sel=2`: given beat address `A` (16-byte aligned, region-relative),
returns two 64-bit entries `{entry(i+1), entry(i)}` with `i = A[11:3]`, `k = A[ADDR_W-1:12]`, little-endian, 2-cycle
latency like the RAMs.

### 5.9 `data_ring`
```
// user data input (data_clk domain)
input data_clk, data_rst; input [127:0] s_data_tdata; input s_data_tvalid; output s_data_tready; (tkeep ignored: full beats only; tlast ignored)
// user_clk domain
output [N_SLOTS-1:0] slot_full;  input [N_SLOTS-1:0] slot_free;  // free = pulse per slot after its CQE
output [$clog2(N_SLOTS):0] slots_used;  output logic input_stalled; // tready low on input (stat)
// completer read port
input  [ADDR_W-1:0] rd_addr; input rd_en; output [127:0] rd_data;  (2-cycle latency, region-relative beat address)
```
Behaviour: async FIFO (depth 512 beats) → sequential write pointer over the BRAM ring (`N_SLOTS·SLOT_BYTES/16`
beats). Slot k becomes `slot_full[k]=1` when its last beat is written. Writing into a slot that is still full
(not yet freed) is blocked → FIFO fills → `s_data_tready=0`. `slot_free[k]` clears the flag. Stats: total beats in,
stall cycles.

### 5.10 `rp_requester`
```
// command port
input  req_valid; output req_ready;
input  [1:0] req_op;      // 0=CFGRD0, 1=CFGWR0, 2=MRD32, 3=MWR32
input  [31:0] req_addr;   // MRd/MWr: byte address (DW aligned). Cfg: {bus[7:0], dev[4:0], fn[2:0], 4'b0, extreg[3:0], reg[5:0], 2'b0} i.e. the H2 layout
input  [31:0] req_wdata;  // little-endian value
input  [3:0]  req_be;     // first BE
output rsp_valid;         // one pulse per command (MWr: immediately after send, status SC)
output [2:0]  rsp_status; // 0=SC, 1=UR, 2=CRS, 3=CA, 4=TIMEOUT, 5=UNEXPECTED(len/tag)
output [31:0] rsp_rdata;
input  [31:0] timeout_cycles;  // parameterizable; default 10 ms @125 MHz = 1_250_000; sims use small values
// streams
output tlp_axis m_req; input m_req_tready;      // to tx arbiter
input  tlp_axis s_cpl; output s_cpl_tready;     // from rx demux (Cpl/CplD)
input  [15:0] requester_id;                     // 16'h0000
```
One outstanding non-posted request at a time; tag increments per request (8-bit). Unexpected completions (tag
mismatch, late) are consumed and counted. For CfgRd/MRd the CplD payload DW is already little-endian (adapter).

### 5.11 `req_mux`
Two command ports with the `rp_requester` command-port signature, fixed priority to port 0 (admin) but port 1
(write-engine doorbells, MWr only) is never blocked for more than one command.

### 5.12 `link_bringup`
```
input  clk (free-running, e.g. 100 MHz from refclk via BUFG or the board clock), rst;
input  user_lnk_up, [5:0] pl_ltssm_state, pl_sel_lnk_rate, [1:0] pl_sel_lnk_width, pl_link_gen2_cap, pl_link_partner_gen2_supported;
output perst_n;          // to SSD
output sys_rst_n;        // to pcie_7x
output link_ready;       // link up and ≥100 ms elapsed since PERST# release
output link_gen2;        // 1 if 5 GT/s was reached (informational)
output [1:0] link_width;
parameter PERST_HOLD_MS = 100, POST_PERST_MS = 100, GEN2_WAIT_MS = 100; parameter CLK_HZ = 100_000_000;
```
FSM: `HOLD` (perst_n=0, sys_rst_n=0 for PERST_HOLD_MS) → `RELEASE` (sys_rst_n=1, then perst_n=1) → `WAIT_LNK`
(user_lnk_up) → `WAIT_100MS` → `WAIT_GEN2` (until `pl_sel_lnk_rate && ltssm==6'h16`, or GEN2_WAIT_MS elapsed) →
`READY`. If `user_lnk_up` drops in READY: `link_ready=0` and go back to HOLD (full retrain with PERST#).

### 5.13 `rp_cfg_init`
```
input start; output done; input [2:0] mps_code; input ext_tag_en;
// pcie_7x cfg_mgmt port
output [31:0] cfg_mgmt_di; output [3:0] cfg_mgmt_byte_en; output [9:0] cfg_mgmt_dwaddr; output cfg_mgmt_wr_en, cfg_mgmt_rd_en;
input  [31:0] cfg_mgmt_do; input cfg_mgmt_rd_wr_done; output cfg_mgmt_wr_readonly = 0, cfg_mgmt_wr_rw1c_as_rw = 0;
```
Sequence: WR dwaddr 0x001 = 0x0000_0006 (BE 0001); WR dwaddr 0x006 = 0x0001_0100 (BE 0111); RD 0x01A → modify
[7:5]=mps_code, [8]=ext_tag_en → WR 0x01A (BE 0011). Each op: assert en with data until `rd_wr_done`.

### 5.14 `pcie_enum`
Uses a `rp_requester` command port. `input start; output done, fail; output [7:0] fail_step; output [2:0] mps_code;
output ext_tag_en; output [31:0] vid_did; output [31:0] bar0_size_mask;` Steps (BDF 01:00.0, CRS retry with 1 ms
delay up to `CRS_RETRY_MAX`=1000; UR/timeout → fail):
1. CfgRd 0x00 → `vid_did` (≠ 0xFFFF_FFFF).
2. CfgRd 0x08 → class; record `class_ok = (val[31:8]==24'h010802)` (warning only).
3. CfgWr 0x10 = 0xFFFF_FFFF; CfgRd 0x10 → `bar0_size_mask`; CfgWr 0x14 = 0xFFFF_FFFF; CfgRd 0x14.
4. CfgWr 0x10 = `BAR0_ADDR`; CfgWr 0x14 = 0.
5. CfgRd 0x34 → ptr; loop: CfgRd ptr → if `[7:0]==0x10` stop, else ptr=`[15:8]` (ptr==0 → fail).
6. CfgRd cap+0x04 (DevCap) → MPSS=[2:0], ETFS=[5]; `mps_code = min(MPSS, MPS_CAP_CODE)`; `ext_tag_en = ETFS & EXT_TAG_ALLOW`.
7. CfgRd cap+0x08 → DevCtl; set [7:5]=mps_code, [14:12]=MRRS_CODE(2 = 512 B), [8]=ext_tag_en; CfgWr cap+0x08 (BE 0011).
8. CfgWr 0x04 = 0x0000_0406 (BE 0011).

### 5.15 `nvme_admin`
Uses a requester command port (port 0 of `req_mux`), writes SQEs into `asq_ram`, polls `acq_ram`, reads `ident_ram`.
```
input start_init; output init_done, init_fail; output [7:0] fail_step; output [15:0] cqe_status_last;
input start_shutdown; output shutdown_done;   // Delete SQ1 → Delete CQ1 → CC.SHN=01 → wait SHST==10
output [3:0] lbads; output [63:0] nsze; output [7:0] mdts; output [3:0] dstrd; output vwc;
input  clear_done; output clear_cqs;            // CQ zero-fill before CC.EN
output [15:0] asq_tail/acq_head (debug)
```
Init FSM (values from research/02 §12): read CAP lo/hi (check MPSMIN=0, CSS[37]); read CC; if EN → write 0, poll
RDY=0; `clear_cqs`; write AQA=0x001F001F, ASQ lo=0x1000_0000 hi=0, ACQ lo=0x1000_1000 hi=0; write CC=0x00460000;
write CC=0x00460001; poll CSTS until RDY=1 (CFS=1 → fail; timeout `(TO+1)·500 ms`); admin commands by sub-FSM
`admin_cmd(sqe[15:0] DWs)`: write 4 beats to `asq_ram[tail]`, MWr `SQ0TDBL` (0x1000) = tail+1, poll `acq_ram[head]`
until `DW3[16]==phase`, check `DW3[31:17]==0` (else fail with `cqe_status_last`), head++ (wrap → phase^=1), MWr
`CQ0HDBL` (0x1000 + (4<<dstrd)) = head. Commands: Identify Controller (CID 0, CNS 1, PRP1 0x1000_2000); extract
MDTS (byte 77), SQES/CQES (512/513 must be 0x66/0x44), VWC (525); Identify Namespace (NSID 1, CNS 0); extract NSZE
(bytes 0–7), FLBAS (26), LBAF[FLBAS&0xF] at 128+4·n → LBADS=[23:16], MS=[15:0] must be 0; Set Features FID 7
(CDW10=7, CDW11=0); Create IOCQ (opc 5, PRP1 0x1000_4000, CDW10 0x003F0001, CDW11 1); Create IOSQ (opc 1, PRP1
0x1000_3000, CDW10 0x003F0001, CDW11 0x00010001). Then `init_done`.
Polling interval for CSTS: every 1 ms. Doorbell addresses computed from `dstrd`.

### 5.16 `nvme_write_engine`
```
input enable;  input [63:0] start_lba; input [3:0] lbads; input [63:0] nsze;
input  [N_SLOTS-1:0] slot_full; output [N_SLOTS-1:0] slot_free;
// iosq_ram write port (4 beats per SQE), iocq_ram read port
// requester command port 1 (MWr doorbells only), input [3:0] dstrd
input  flush_req; output flush_done;     // submit Flush when all slots are free, wait its CQE
output [63:0] next_lba, bytes_done, cmds_done; output [31:0] cmd_errors; output [15:0] last_status;
output [31:0] max_latency_cycles; output sq_full_stall (stat); output [6:0] sq_tail, cq_head (debug)
```
Submit path: `k = next_slot`; if `slot_full[k] && !busy[k] && !sq_full && enable`: write SQE into `iosq_ram[tail]`:
DW0=`{16'(k), 8'h0, 8'h01}`, DW1=1, DW2..5=0, DW6=`DATA_BASE + k·SLOT_BYTES`, DW7=0, DW8=`PRP_BASE + k·4096`, DW9=0,
DW10/11=`next_lba`, DW12=`(SLOT_BYTES>>lbads)-1`, DW13=`32'h40` (sequential hint), DW14/15=0. Then tail++ (mod 64),
MWr `SQ1TDBL` (0x1000 + (2·1)·(4<<dstrd)) = tail; `busy[k]=1`; `next_lba += SLOT_BYTES>>lbads`; `next_slot++`.
Guard: `next_lba + nlb > nsze` → stop with `lba_range` error flag (do not submit).
Reap path: continuously read `iocq_ram[head]`; when `DW3[16]==phase`: `cid=DW3[15:0]` → `slot_free[cid]` pulse,
`busy[cid]=0`; `status=DW3[31:17]` (≠0 → `cmd_errors++`, `last_status`); head++ (wrap → phase^=1); MWr `CQ1HDBL`
= head. Also handle the Flush CQE (CID = 0xFF). Latency: timestamp per slot at doorbell; max over run.
Note the RAM read latency: after head advances, wait 2 cycles before trusting the read (avoid reading stale entry).

### 5.17 `host_sequencer`
```
input start, stop;   // level: start begins the whole bring-up; stop requests orderly shutdown
output [3:0] state; output [7:0] error_code; output running, done;
```
States: IDLE → LINK (wait link_ready) → ENUM → RPCFG → ADMIN_INIT → RUN (write_engine enable) → DRAIN (stop: wait
all slots free, flush) → SHUTDOWN (nvme_admin shutdown) → DONE. Any fail → ERROR (error_code = step id).

### 5.18 `nvme_host_top` ports
```
input  sys_clk_p, sys_clk_n;            // 100 MHz PCIe refclk (shared with SSD)
input  board_rst_n;                     // board reset (active low)
output pcie_perst_n;                    // to M.2 PERST#
output [3:0] pci_exp_txp, pci_exp_txn; input [3:0] pci_exp_rxp, pci_exp_rxn;
input  data_clk; input [127:0] s_data_tdata; input s_data_tvalid; output s_data_tready;
input  ctrl_start, ctrl_stop;
output status_link_up, status_gen2, status_enum_done, status_nvme_ready, status_running, status_done;
output [7:0] status_error; output [63:0] stat_bytes_done; output [31:0] stat_cmd_errors; output [31:0] stat_max_latency;
output user_clk_out (125 MHz, for user's convenience)
```

---

## 6. Parameters (package `nvme_host_pkg`)

| Name | Default | Meaning |
|---|---|---|
| `N_SLOTS` | 8 | ring slots (= max queue depth) |
| `SLOT_BYTES` | 131072 | bytes per command (must be ≤ 2^MDTS pages, multiple of 4096) |
| `MPS_CAP_CODE` | 1 (256 B) | our maximum MPS; must equal the pcie_7x GUI "Max Payload Size" |
| `MRRS_CODE` | 2 (512 B) | programmed into the SSD |
| `EXT_TAG_ALLOW` | 1 | enable extended tags if SSD supports |
| `BAR0_ADDR` | 32'hC000_0000 | |
| `ASQ_BASE, ACQ_BASE, IDENT_BASE, IOSQ_BASE, IOCQ_BASE, PRP_BASE, DATA_BASE` | see §2 | |
| `ADMIN_QSIZE` | 32 | admin queue entries |
| `IO_QSIZE` | 64 | I/O queue entries |
| `REQ_TIMEOUT_CYCLES` | 1_250_000 | 10 ms at 125 MHz; testbench overrides via parameter |
| `SIM_FAST` | 0 | shrinks millisecond timers to microseconds in simulation |

---

## 7. Verification strategy (xsim, Vivado 2022.2)

`sim/models/`:
- `pcie7x_core_model.sv` — mimics the core-facing side: drives `m_axis_rx` in 128-bit format with **randomised SOF
  at DW0/DW2, straddling, and 3DW misalignment exactly per PG054**; consumes `s_axis_tx` (checks tkeep/tlast rules,
  no tvalid drop mid-packet); random `tready` deassertion; `cfg_mgmt` register file (Command, DevCtl, bus numbers);
  emulates "BME=0 → upstream MRd answered with UR / MWr dropped" and "payload > DevCtl.MPS → malformed error".
- `nvme_ssd_model.sv` — behavioural NVMe controller behind the core model: config space (VID/DID, class, 64-bit
  BAR0 of 16 KiB, cap list with PCIe cap at 0x80 incl. DevCap MPSS=2 / ETFS=1, DevCtl), CRS for the first N config
  requests, BAR0 registers (CAP with MPSMIN=0, TO=1, DSTRD=0, MQES=1023; VS; CC; CSTS with RDY following EN after a
  delay; AQA/ASQ/ACQ; doorbells), admin command processing (Identify Controller/Namespace with realistic data incl.
  MDTS=5, LBADS=9, NSZE; Set Features; Create CQ/SQ; Delete; and shutdown SHST), I/O Write processing: fetch SQE
  (MRd 64 B), fetch PRP list (random chunking), fetch data with **up to 16 outstanding MRds of random size ≤ MRRS**,
  reassemble CplDs **checking BC/LA/Length/ordering rules**, verify data against a reference model (testbench
  pushes the expected stream), post CQEs (sometimes two per MWr, sometimes split 8+8 with P last), Flush, status
  injection for error tests. Writes to the LBA space are checked for sequential coverage.
- Scoreboard: every byte streamed in must land at LBA `start_lba + offset/512` exactly once.

`sim/tb/`: `tb_rx_adapt`, `tb_tx_adapt`, `tb_completer`, `tb_writer`, `tb_data_ring`, `tb_requester`, `tb_top`
(full flow: link → enum → init → stream 4 MiB → stop → shutdown; prints achieved beats/cycle on the TX bus as the
throughput metric; must reach ≥ 0.9 beats/cycle during steady state with the model's tready always high).
`sim/scripts/run_xsim.ps1` and `run_xsim.sh` run all tests via `xvlog -sv`, `xelab`, `xsim -R`; exit code non-zero
on any `$error`/`$fatal`. `SIM_FAST=1` for all tests.

---

## 8. Throughput budget (design targets)

- TX adapter + completer: 1 beat/cycle sustained → 2.0 GB/s bus; CplD framing 64/68 → 1.88 GB/s; link framing with
  MPS=256 → ≈1.82 GB/s; realistic 1.6–1.75 GB/s (research/02 §7).
- Completer latency (MRd header in → first CplD beat out): ≤ 12 cycles (≈100 ns).
- Data ring must absorb ≥ 500 µs of SSD stall at full rate: 8 slots × 128 KiB = 1 MiB ≈ 580 µs at 1.8 GB/s. ✔

---

## 9. Educational deliverables (docs/) mapping

| Chapter | Source |
|---|---|
| 1 PCIe с нуля | general + research/03 |
| 2 NVMe с нуля | research/02 |
| 3 Архитектура хоста | this document §1–2, §4, §6, §8 |
| 4 Модули и интерфейсы | §3, §5 with timing diagrams from `docs/wave/*.json` |
| 5 Запуск по шагам | §5.12–5.15, research/02 §12 |
| 6 Пропускная способность | §8, research/02 §7 |
| 7 Верификация | §7 |
| 8 Лабораторные задания | derived (e.g. implement MRRS=4096 splitting, add DDR ring, add MSI, add Read engine) |
