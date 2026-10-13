# Brief: Kintex-7 `pcie_7x` v3.3 as a Root Port for an NVMe SSD, compared with `axi_pcie`

> Research report produced by an Opus 5.5 research agent (2026-10-14) from PG054 v3.3 (Dec 2022), the
> IP templates shipped in Vivado 2022.2 (`C:\Xilinx\Vivado\2022.2\data\ip\xilinx\pcie_7x_v3_3\ttcl\`),
> and PG055. Items marked **[UNVERIFIED]** are inferences.

---

## 1. Supported configurations on Kintex-7 (PG054 Table 2-4 p.11, Table 2-8 p.17, xgui tcl)

- **Speeds.** The core supports Gen1 and Gen2 only, x1 to x8, in both Endpoint and Root Port mode. **Gen3 is not available on Kintex-7.**
- **Speed grade.** x8 Gen2 needs -2/-3/-2L; x1–x4 Gen2 also works on -1. XC7K325T has one PCIe block, `PCIE_X0Y0`.

**Data widths and user clock (`user_clk_out`):**

| Link | AXI-S width | `user_clk` |
|---|---|---|
| Gen2 x4 | 64-bit | **250 MHz** (default) |
| Gen2 x4 | 128-bit | **125 MHz** |
| Gen2 x8 | **128-bit only** | **250 MHz only** |
| Gen1 x8 | 64-bit | 250 MHz |
| Gen1 x8 | 128-bit | 125 MHz (250 optional) |

- **Max payload size (MPS) capability.** 128, 256, 512 or 1024 bytes. For x8 Gen2 the GUI limits it to 256 B on -2 and 512 B on -3.
- **NVMe SSDs are x4.** Use a Gen2 x4 core.
- **Throughput.** Gen2 x4 is 2.0 GB/s per direction before packet overhead. With MPS = 256 B, expect roughly 1.7–1.8 GB/s of payload. Both 64b@250 and 128b@125 reach about 94% beat efficiency for 256 B completions.

## 2. TLP format on the 7-series AXI-Stream interface (raw TLPs, not descriptors)

### Byte and dword ordering (this is the most important detail)

- **Header dword placement.** Header DW0 sits in `tdata[31:0]`, DW1 in `[63:32]`, DW2 in `[95:64]` (128-bit mode), and so on. Within each dword, bits follow the PCIe specification: `tdata[30:29]` = Fmt, `[28:24]` = Type, `[22:20]` = TC, `[15]` = TD, `[14]` = EP, `[13:12]` = Attr, `[9:0]` = Length. In DW1: Requester ID `[63:48]`, Tag `[47:40]`, Last BE `[39:36]`, First BE `[35:32]`.
- **Payload dwords use PCIe wire byte order, i.e. big-endian within each dword.** The byte at the lowest address (enabled by BE[0]) is in **bits [31:24]** of the payload dword.
  - Evidence 1: the shipped config ROM writes Command = 0x0006 using data `h06000000`, and BAR = 0x8000_0000 using `h00000080`.
  - Evidence 2: `PIO_EP_MEM_ACCESS` sets `w_wr_data_b0 = wr_data[31:24]` and returns `rd_data = {byte0, byte1, byte2, byte3}`.
- **Consequence for NVMe.** NVMe structures in memory are little-endian, so **byte-swap every payload dword** at the TLP boundary.
- **`cfg_mgmt_di` / `cfg_mgmt_do` are not swapped.** They carry normal little-endian register values (PG054 p.112–113).

### Transmit: `s_axis_tx_*` (Table 2-9 pp.17–19)

- **`tuser[3:0]`:** `[0]` `tx_ecrc_gen`, `[1]` `tx_err_fwd` (poison), `[2]` `tx_str` (streaming / cut-through), `[3]` `tx_src_dsc` (discontinue).
- **`tkeep` on non-last beats** must be all ones. Last beat, 64-bit: `0x0F` or `0xFF`. Last beat, 128-bit: `0x000F`, `0x00FF`, `0x0FFF` or `0xFFFF`.
- **128-bit TX has no straddling.** Every TLP starts at DW0 of a new beat (p.68).
- **Streaming mode (`tx_str` = 1).** `tvalid` must not drop in the middle of a packet; otherwise the TLP is nullified and `tx_err_drop` is raised (p.52). The Xilinx examples use `tuser = 4'b0100`.
- **Acceptance guarantee.** Once the core accepts the first beat, it will accept the whole TLP up to the capability MPS (p.52). The core drops a TLP whose payload exceeds the Device Capability MPS.
- **When the core drops `tready`:** after a packet completes when its buffers are full of un-ACKed TLPs or it lacks link-partner credits; while it sends a TLP of its own after `tx_cfg_gnt`; in non-D0 power state. User logic must hold data stable while `tready` is low (p.51–52).
- **`tx_buf_av[5:0]`** = number of free TLP buffers, one TLP per buffer regardless of size (Table 3-13 p.93): "high" performance level: 32 buffers at MPS 128, 29 at 256. Posted and completion TLPs can overtake *blocked* non-posted TLPs only if already loaded into the core. Watch `tx_buf_av` so a completion always has a buffer (p.94).
- **`tx_cfg_req` / `tx_cfg_gnt`.** The core raises `tx_cfg_req` when it has an internally generated TLP to send (e.g. automatic UR completion). **Tie `tx_cfg_gnt` = 1** unless you need priority; never hold it low indefinitely (p.19, p.48).
- **ACK/NAK, LCRC, replay, sequence numbers, replay timer and transmit flow-control credits are all handled inside the core** (Table 3-42 p.134). **User logic only has to obey `tready`.**

### Receive: `m_axis_rx_*` (Table 2-10 pp.20–23)

- **`tuser[21:0]`:**

| Bits | Name | Notes |
|---|---|---|
| `[0]` | `rx_ecrc_err` | Valid at EOF |
| `[1]` | `rx_err_fwd` | Packet is poisoned. Held for the whole packet; on a straddled beat applies to the packet that is starting |
| `[9:2]` | `rx_bar_hit` | **No BAR decode in Root Port mode** (p.64) |
| `[14:10]` | `is_sof` | 128-bit only |
| `[16:15]` | — | Reserved |
| `[21:17]` | `is_eof` | 128-bit only |

- **64-bit mode.** `tlast` and `tkeep` are used; last beat `tkeep` is `0x0F` or `0xFF`. No straddling. `tvalid` never drops mid-packet.
- **128-bit mode** (Table 2-10 p.21, Tables 3-4 to 3-9 pp.77–79, Fig 3-49 p.83):
  - **`tlast` is tied low.** Use `is_sof` / `is_eof` instead.
  - **`is_sof`:** `5'b10000` = SOF at DW0; `5'b11000` = SOF at DW2; `5'b0xxxx` = no SOF. Decode `is_sof[3]`.
  - **`is_eof`:** `5'b10011` = EOF at DW0, `5'b10111` = DW1, `5'b11011` = DW2, `5'b11111` = DW3. Decode `is_eof[3:2]`.
  - **Straddling.** In one beat, one TLP can end in the lower quad-word (EOF at DW0 or DW1) while the next starts at DW2. At most one SOF and one EOF per beat. A packet can also start at DW2 without a straddle.
- **Payload alignment you must handle on RX:**
  - 64-bit mode: 3DW header → payload starts at the upper dword of beat 1 (`{D0,H2}`), misaligned; 4DW header → QW-aligned.
  - 128-bit mode:

| Header | SOF position | Payload starts at |
|---|---|---|
| 3DW | DW0 | DW3 of the SOF beat (`{D0,H2,H1,H0}`) |
| 4DW | DW0 | DW0 of the next beat |
| 3DW | DW2 | DW1 of the next beat (`{D2,D1,D0,H2}`) |
| 4DW | DW2 | DW2 of the next beat (`{D1,D0,H3,H2}`) |

  - Build a realigner. **SSDs use 3DW MWr/MRd for addresses below 4 GB**, so the misaligned 3DW case is the normal one.

**Stalling `m_axis_rx_tready`.** User logic may deassert it at any time, including mid-packet, without limit (p.58). Long stalls fill the RX buffer, credits stop returning, the SSD stalls.

**`rx_np_ok`** (Receive Non-Posted Throttling, default mode). Deassert it at least 2 clocks before `tlast` of the second-to-last non-posted TLP you can accept. Posted and completion TLPs then bypass queued non-posted reads (pp.60–61, p.84). Useful for NVMe: the SSD's MWr/CQEs keep flowing while the read engine is busy. The alternative `rx_np_req` mode is a GUI option; mutually exclusive.

## 3. TLP header encodings and examples

### Fmt/Type byte (`DW0[31:24]`)

| TLP | Fmt[1:0] / Type[4:0] | `DW0[31:24]` |
|---|---|---|
| MRd32 (3DW) | 00 / 00000 | 0x00 |
| MRd64 (4DW) | 01 / 00000 | 0x20 |
| MWr32 (3DW + data) | 10 / 00000 | 0x40 |
| MWr64 (4DW + data) | 11 / 00000 | 0x60 |
| CfgRd0 | 00 / 00100 | 0x04 |
| CfgWr0 | 10 / 00100 | 0x44 |
| CfgRd1 / CfgWr1 | 00 or 10 / 00101 | 0x05 / 0x45 |
| Cpl (no data) | 00 / 01010 | 0x0A |
| CplD | 10 / 01010 | 0x4A |
| Msg / MsgD | 01 or 11 / 10rrr | 0x30–0x37 / 0x70–0x77 |

- **DW0 low fields.** TC `[22:20]`, TD `[15]`, EP `[14]`, Attr `[13:12]`, Length `[9:0]` in dwords (0 = 1024).
- **Memory request.** DW1 = {ReqID[31:16], Tag[15:8], LastBE[7:4], FirstBE[3:0]}. 3DW: DW2 = {Addr[31:2], 00}. 4DW: DW2 = Addr[63:32], DW3 = {Addr[31:2], 00}. Use 4DW only for addresses ≥ 4 GB.
- **Config Type 0 request.** DW1 = {ReqID, Tag, LastBE = 0000, FirstBE}. DW2 = {Bus[31:24], Dev[23:19], Fn[18:16], 0000, ExtReg[11:8], Reg[7:2], 00}. Length must be 1 and TC/Attr 0 (p.131).
- **Completion.** DW1 = {CompleterID[31:16], Status[15:13], BCM[12], ByteCount[11:0]}. DW2 = {ReqID[31:16], Tag[15:8], 0, LowerAddr[6:0]}. Status: 000 = SC, 001 = UR, **010 = CRS**, 100 = CA.

### Example: MWr32, 1 DW payload

Write little-endian 0x0000_0001 to 0xE000_1000 (NVMe SQ0 tail doorbell if BAR0 = 0xE000_0000). ReqID = 0x0000, Tag = 0, FirstBE = F.

- DW0 = `0x4000_0001`, DW1 = `0x0000_000F`, DW2 = `0xE000_1000`, Payload DW = byte-swap(0x00000001) = `0x0100_0000`

**64-bit mode:**

| Beat | `tdata[63:32]` | `tdata[31:0]` | `tkeep` | `tlast` |
|---|---|---|---|---|
| 0 | `0x0000_000F` (DW1) | `0x4000_0001` (DW0) | `0xFF` | 0 |
| 1 | `0x0100_0000` (D0) | `0xE000_1000` (DW2) | `0xFF` | 1 |

**128-bit mode (single beat):** `tdata = 0x0100_0000_E000_1000_0000_000F_4000_0001` ([127:96] = D0, [95:64] = DW2, [63:32] = DW1, [31:0] = DW0), `tkeep = 0xFFFF`, `tlast = 1`.

**MWr64 variants with 1 DW payload:** 64-bit mode: `{DW1,DW0}`, `{DW3,DW2}`, `{--,D0}` with `tkeep = 0x0F`. 128-bit mode: `{DW3,DW2,DW1,DW0}`, then `{--,--,--,D0}` with `tkeep = 0x000F`.

### Example: CfgRd0 of Vendor/Device ID, bus 1, dev 0, fn 0, tag 1, RP ID 0x0000

- DW0 = `0x0400_0001`, DW1 = `0x0000_010F`, DW2 = `0x0100_0000`.
- 64-bit mode: beat 0 = `0x0000010F_04000001`; beat 1 = `{xxxx, 0x01000000}` with `tkeep = 0x0F`, `tlast` = 1.
- **The returning CplD** for a Samsung part: DW0 = `0x4A00_0001`; DW1 = {CplID = 0x0100, SC, BC = 4} = `0x0100_0004`; DW2 = `0x0000_0100`. Data arrives byte-swapped: DID/VID 0xA808_144D shows up as `0x4D14_08A8`.
- **CfgWr0 to the Command register** with MSE + BME + INTx-disable = 0x0406, BE = 0011: DW0 = `0x4400_0001`, DW1 = `0x0000_0203` (tag 2), DW2 = `0x0100_0004`, Data = `0x0604_0000`.

## 4. Root Port specifics

### What the core does and does not do in Root Port mode

**Receive dispositions (Table 3-12 pp.91–92):**

| Received TLP | Core response |
|---|---|
| Upstream MRd/MWr | No BAR filtering; passed to the user, **if Command.BME = 1** |
| Completions | Passed to the user, with no Requester ID filtering |
| CfgRd0/CfgWr0 | UR |
| CfgRd1/CfgWr1 | Passed to the user |

- **No Requester ID check on completions.** The Xilinx Root Port example uses arbitrary requester IDs (cgator `16'h10EE`, pio_master `16'hFACE`).
- **No Base/Limit filtering.** PG054 p.48 and p.67. Do your own filtering if you want it.
- **Command register in Root Port mode (Table 3-25 pp.109–110):** I/O and Memory Space Enable are ignored for downstream traffic. **Bus Master Enable = 0 makes the Root Port answer upstream Mem/IO requests with UR** (Cpl-UR for MRd, MWr dropped). **You must set the Root Port's own BME through `cfg_mgmt`.** Listed in Xilinx's PCIe Debug K-Map as a common mistake.
- **PG054 p.112:** "Root Ports must use the Configuration Port to set up the Configuration Space."
- **The core has no completion timers** (p.128), does not generate completions for memory reads (p.121), and does not track your outstanding non-posted requests.
- **Receive checks the core makes (Table 3-42 pp.130–133):** payload ≤ MPS; completion data ≤ MPS; MWr does not cross a 4 KB boundary; Length matches payload; TD matches digest.
- **Receive checks left to you:** RCB violations, unexpected completions, poisoned TLPs.

### Minimal `cfg_mgmt` writes to the Root Port

The Root Port Type 1 header and its PCIe capability are at **0x60** (`PCIE_BASE_PTR = 8'h60`). Hold `cfg_mgmt_wr_en` with address, data and byte-enable until `cfg_mgmt_rd_wr_done` (Fig 3-61 p.113). `cfg_mgmt_dwaddr` is a dword address.

| Register | `dwaddr` | Value / BE | Required? |
|---|---|---|---|
| Command (0x04) | 0x001 | `0x0000_0006` (MSE+BME), BE = 0001 | **YES** (BME) |
| Device Control (0x68) | 0x01A | read-modify-write: MPS[7:5] = SSD MPS (001 = 256 B); bit 8 Extended Tag Enable if the SSD uses 8-bit tags; BE = 0011 | **Strongly recommended** (note 1) |
| Bus numbers (0x18) | 0x006 | `0x0001_0100` (primary 0, secondary 1, subordinate 1), BE = 0111 | Hygiene only (note 2) |
| Memory Base/Limit (0x20) | 0x008 | covering the SSD BAR | Not used by the core (p.48) |
| Link Control (0x70) | 0x01C | bit 6 Common Clock if the reference clock is shared | Optional |

1. **Device Control MPS.** The core rejects received TLPs/completions whose payload exceeds "Max_Payload_Size" as malformed (p.130, p.133). Reset value of Device Control MPS is 128 B. Inferred that the check uses the programmed Device Control value **[UNVERIFIED]**. Program the Root Port and the SSD to the same MPS.
2. **Bus numbers.** PG054 documents no internal use; the Xilinx example never writes `cfg_mgmt` at all.
3. **Extended tags.** Tag-length checking is listed as an Integrated Block check (p.131); which Device Control setting governs it is not documented **[UNVERIFIED]**. If you enable Extended Tag on the SSD, enable it on the Root Port too (this write or GUI option `Extended_Tag_Default`).

**Other Root Port ports and their tie-offs** (as in the Xilinx example): `cfg_ds_bus_number/device/function` = 0 (used only in core-generated TLPs, p.31); `cfg_pm_send_pme_to` = 0; `pl_transmit_hot_rst` = 0; `pl_downstream_deemph_source` = 0; `cfg_interrupt*` = 0; `cfg_trn_pending` = 0.

### Config TLP rules

- **Requester ID of the Root Port.** Free field; use 0x0000 (bus 0, dev 0, fn 0). Match completions on your own ReqID and Tag.
- **Target.** Type 0 to bus 1, dev 0, fn 0.
- **SSD requester ID.** A PCIe 3.0+ SSD captures its bus/device number on the first CfgWr0 it receives; afterwards its upstream TLPs carry ReqID 0x0100. Before that they may carry 0x0000; the Root Port does not care.
- **Timing.** Wait **≥ 100 ms after PERST# release** before the first config request. The device may answer **CRS (status 010)** for up to ~1 s; re-issue after a short delay. cgator re-issues immediately with no timeout (p.289). `ROOT_CAP_CRS_SW_VISIBILITY` is irrelevant in raw-TLP mode.
- **UR on a config read** means no device is present.

## 5. Upstream requests from the SSD: you are the completer

- MRd and MWr from the SSD to "host memory" arrive on `m_axis_rx` with no BAR or address filtering, provided Root Port BME = 1. MSI is just a MWr; INTx arrives as messages (section 8).

**Completion rules you must implement** (the core neither generates nor checks them, p.121, p.133):

- CplD: Fmt/Type 0x4A, Status = SC, Completer ID = Root Port ID. Copy ReqID, Tag, TC, Attr from the request. Length = dwords in this completion.
- **Byte Count** = bytes remaining including this completion (0 = 4096). For 1-DW reads use the First/Last BE table (`PIO_TX_ENGINE` has it).
- **Lower Address [6:0].** First completion: address of the first enabled byte. Later completions: `addr[6:0]` of their RCB-aligned start.
- **Splitting.** Each completion payload ≤ MPS (Device Control). Split only at naturally aligned **RCB** boundaries (GUI `LINK_CONTROL_RCB`, default 0 = 64 B; visible on `cfg_lcommand[3]`). Splitting at MPS-aligned addresses (256 B) is always legal. Completions for one request go in increasing address order; different tags may interleave.
- **Worked example.** MRd 512 B at 0x…040, MPS 256, RCB 64:

| CplD | Address range | Size | BC | LA |
|---|---|---|---|---|
| #1 | 0x040–0x0FF | 192 B | 512 | 0x40 |
| #2 | 0x100–0x1FF | 256 B | 320 | 0x00 |
| #3 | 0x200–0x23F | 64 B | 64 | 0x00 |

- **Zero-length read** (Length = 1, BE = 0): reply with a 1-DW CplD with Byte Count = 1.
- **Tag queueing.** The core's RX buffer advertises only **12 non-posted header credits** (Table 3-14 p.95). The SSD's many outstanding reads must be pulled out promptly and queued inside your logic.
- **SSD Device Control MRRS.** Match your completer buffering; 512 B is a reasonable start. Also program the SSD's MPS.

**Transmit acceptance.** Whenever `tready` = 1 and you present a legal TLP ≤ capability MPS, it will be sent reliably.

## 6. Link bring-up as Root Port

- **Clocks.** 100 MHz reference clock, shared with the SSD, into `IBUFDS_GTE2` → `sys_clk`. GTs use the CPLL. The `pipe_clock` block runs one MMCM from lane-0 TXOUTCLK to produce pclk 125/250 MHz, userclk1/2, DRP clock (5–6 BUFGs). Use common reference clocking.
- **Resets.** `sys_rst_n` is an asynchronous active-low hard reset of core and GTs. **The FPGA drives PERST# to the SSD.** Sequence: PERST# low ≥ 100 ms after power stable; refclk stable ≥ 100 µs before releasing PERST#; release `sys_rst_n` then PERST#; SSD must train within 20 ms (p.155); first config request ≥ 100 ms after PERST# release.
- **Status signals.** `user_lnk_up` (stays high through Recovery); `pl_ltssm_state[5:0]`: 0x16 = L0; `pl_sel_lnk_rate`: 1 = 5 GT/s; `pl_sel_lnk_width[1:0]`: 10 = x4; `pl_initial_link_width`, `pl_link_gen2_cap`, `pl_link_partner_gen2_supported`; `cfg_pcie_link_state[2:0]`: 000 = L0.
- **Getting to Gen2.** The Root Port **up-trains to Gen2 automatically** (`RP_AUTO_SPD = 2'h1` default). Do **not** drive `pl_directed_link_change` = 10/11 on a Root Port unless `RP_AUTO_SPD = 11` (p.27, p.102). The Xilinx example waits for `pl_sel_lnk_rate && pl_ltssm_state == 6'h16` before pulsing `start_config` when both sides are Gen2-capable; otherwise it starts on the `user_lnk_up` edge. **No `cgator_gen2_enabler` file exists in v3.3**; the hardware performs the upgrade.

## 7. Max payload and max read request

- **Root Port capability MPS** (GUI): 128–1024 B. Root Port's Device Control MPS resets to 128 B; program it through `cfg_mgmt` (`dwaddr` 0x1A, bits [7:5]).
- **SSD MPS.** CfgWr0 to the SSD's PCIe capability Device Control (walk the capability list from 0x34).
- **Use the smaller of the two capabilities on both sides.** 256 B is the practical choice.
- **MRRS.** The Root Port's own MRRS is irrelevant. The SSD's MRRS (Device Control [14:12]) sets the largest MRd you must service.
- **`cfg_dcommand[15:0]`** mirrors the Root Port Device Control: MRRS [14:12], ExtTag [8], MPS [7:5] (Table 3-20 p.107).

## 8. Interrupts

- Polling the completion queue in memory is fine.
- **An MSI from the SSD is an ordinary MWr** on `m_axis_rx`; absorb or drop it. MSI/MSI-X are disabled after reset. To suppress INTx, set **Command bit 10** on the SSD; NVMe INTMS can mask too.
- **INTx, ERR_* and PME messages** are reported on `cfg_msg_received` plus `cfg_msg_received_assert_inta`/`_deassert_inta`/`_err_cor`/`_err_non_fatal`/`_err_fatal`/`_pm_pme`/`_pme_to_ack`, requester ID on `cfg_msg_data[15:0]` (Table 2-16). By default not routed to AXI-S (p.121–122); GUI "Enable Message Routing" forwards them too. No stall either way.

## 9. Completion timeout, errors, ECRC and poison: minimal safe behavior

- **Root-Port-initiated requests.** Keep your own per-tag timer (50 µs–50 ms per PCIe; PG054 recommends ≥ 10 ms). For config requests allow ~1 s of CRS retries. Discard late completions as unexpected.
- **ERR_FATAL / ERR_NONFATAL** from the SSD on `cfg_msg_received_err_*`: log, treat as fatal to the session.
- **`cfg_err_*` inputs.** Tie to 0 (as the Xilinx example does). Optionally pulse `cfg_err_cpl_timeout` / `cfg_err_cpl_unexpect`.
- **Poisoned TLPs (`rx_err_fwd`).** MWr: drop and flag. CplD: fail the read. Never transmit poisoned TLPs; `tx_err_fwd` = 0.
- **ECRC.** AER ECRC off by default; treat `rx_ecrc_err` like poison. `tx_ecrc_gen` = 0.
- **Handled by the core:** link-level errors, LCRC, replay; malformed-TLP checks; Cpl-UR for upstream requests when BME = 0; Cpl-UR for received CfgRd0/CfgWr0.

## 10. Example design and the cgator "Configurator" (PG054 pp.286–292)

**Availability.** Vivado 2022.2 ships it: generate a Root Port `pcie_7x` core, then "Open IP Example Design".

| File | Role |
|---|---|
| `xilinx_pcie_2_1_rport_7x.v` | Top level |
| `cgator_wrapper.v`, `cgator.v` | Configurator |
| `cgator_controller.v` | Steps through the ROM |
| `cgator_pkt_generator.v` | Generates the TLPs |
| `cgator_tx_mux.v` | Arbitrates TX between cgator and user |
| `cgator_cpl_decoder.v` | Decodes completions |
| `cgator_cfg_rom.data` | The configuration ROM |
| `pio_master*.v` | PIO master test logic |

Templates: `ttcl\rp_cgator_*_vlog.ttcl`, `ttcl\rp_pio_master_*_vlog.ttcl`.

**How it works.** Pulse `start_config` once after link-up. The controller reads ROM entries in pairs and sends one TLP per pair. For CfgRd/CfgWr it waits for the completion: SC → next; **CRS → re-send**; UR/CA/ReqID mismatch → `failed_config`. **No completion timeout**; completion data ignored. Ends with `finished_config`. Ties `m_axis_rx_tready` = 1 and `rx_np_ok` = 1. Never touches `cfg_mgmt`. In 128-bit mode the completion decoder ignores SOF at DW2 (OK only because one TLP outstanding).

**ROM format.** `$readmemb`, binary, `//` comments. Entries alternate: even = control word (`[17:16]` type: 00 CfgRd, 01 CfgWr, 10 Msg, 11 MsgD; for Cfg `[15:14]` fn, `[13:4]` dword register number, `[3:0]` First BE; for Msg `[10:8]` routing, `[7:0]` code); odd = 32-bit data in TLP byte order (Command = 0x0006 → `00000110 00000000 00000000 00000000`). `ROM_SIZE` = 2 × TLPs (default 32), `REQUESTER_ID` default `16'h10EE`. Target hard-coded bus 1, dev 0.

**Shipped ROM sequence (16 TLPs):** Set_Slot_Power_Limit MsgD (25 W); CfgRd Device ID; BAR0/1/2/ExpROM sizing; BAR assignment (64-bit BAR0/1 = `0x1000_0000_8000_0000`, BAR2 = `0x2000_0000`); Link Control = 0x41; Command = 0x0006.

**Notes for reuse:** copy the ROM-driven idea, add a timeout and a CRS retry delay. **Do not copy `cgator_tx_mux` as is** (not a proper AXI-Stream skid buffer).

**Resources** (core only): ~1.8–1.9k LUTs / 2.0–2.1k FFs for x4 Gen2; 2–16 BRAMs; 1 MMCM; 5–6 BUFGs.

## 11. Gen2 GTX placement and clock pins

- **XC7K325T-FFG900 with `PCIE_X0Y0`.** Default XDC places lane *i* on `GTXE2_CHANNEL_X0Y(7−i)`: x4 uses X0Y7–X0Y4.
- **KC705 edge connector** is Endpoint form factor (refclk and PERST# come from the host). **For a Root Port to an SSD you need an adapter that supplies the reference clock.**
- **Practical KC705 option: Opsero "FPGA Drive FMC" on HPC.** Its `kc705_hpc.xdc` **[UNVERIFIED in detail]**: refclk on GBTCLK0_M2C pins C8/C7 (100 MHz); PERST# on LA00_CC_P pin C25 LVCMOS25; lanes 0–3 on `GTXE2_CHANNEL_X0Y12..X0Y15`; `PCIE_X0Y0`.

## 12. `axi_pcie` (PG055) Root Port alternative

- Kintex-7 supported in Root Port mode (C_INCLUDE_RC). x8 is Gen1 only; Gen2 limited to x1/x2/x4. AXI width 64/128; x4 Gen2 is 128-bit only at 125 MHz. **MPS limited to 256 B.** 32-bit AXI addresses. No AER/MSI-X.
- **Slave bridge (AXI → PCIe):** ≤ 2 outstanding writes, ≤ 4 reads (128-bit Gen2 7-series). **Bridge Enable** (0x148 bit 0) must be set after enumeration.
- **Master bridge (PCIe → AXI):** **≤ 4 outstanding MWr and 4 MRd** with pending completions. Generates completions split at MPS with RCB observed. Single RP BAR; "Hide RP BAR" + 64-bit 4 GB BAR to accept every address.
- **ECAM window** on AXI4-Lite: [7:2] register, [11:8] ext register, [14:12] function, [19:15] device, [20+] bus. Bus 0 = Root Port's own Type 1 header. Blocking; UR → all-ones.
- **Trade-off.** Much simpler but the 4-deep master-bridge pipeline, 256 B MPS cap and AXI burst conversion limit throughput. No authoritative MB/s figures found **[UNVERIFIED]**.

**Recommendation.** For maximum throughput and deterministic streaming, **use raw `pcie_7x` at Gen2 x4** with a custom TLP completer (answers from FIFOs/BRAM directly), a custom MWr sink and a small cgator-like config engine. This saturates the link at ~1.7–1.8 GB/s with deep tag queueing. Choose `axi_pcie` only if development time matters more than bandwidth or a soft CPU will run a driver.

---

## Not verified / caveats

1. Whether the core's receive MPS check compares against Device Control (programmed) or Device Capability.
2. Whether the Root Port uses the secondary/subordinate bus registers internally at all.
3. Which Extended Tag setting governs the tag-length check on received requests.
4. `cfg_pm_send_pme_to` polarity.
5. KC705 / Opsero XDC pin details.
6. `axi_pcie` throughput numbers.
7. PG054 Table 3-12 text extraction was jumbled around message rows.
8. PCIe timing rules (100 ms, 1 s CRS, 50 µs–50 ms timeout, T_PERST-CLK 100 µs) come from PCIe Base/CEM as known, not from PG054.

**Links:** [PG054 v3.3](https://www.xilinx.com/support/documents/ip_documentation/pcie_7x/v3_3/pg054-7series-pcie.pdf) · [PG055 v2.8](https://www.xilinx.com/content/dam/xilinx/support/documents/ip_documentation/axi_pcie/v2_8/pg055-axi-bridge-pcie.pdf) · [Opsero fpga-drive-aximm-pcie](https://github.com/fpgadeveloper/fpga-drive-aximm-pcie) · [Xilinx PCIe Debug K-Map](https://xilinx.github.io/pcie-debug-kmap/pciedebug/build/html/docs/PCIe_Common_Issues/index.html) · local templates `C:\Xilinx\Vivado\2022.2\data\ip\xilinx\pcie_7x_v3_3\ttcl\`
