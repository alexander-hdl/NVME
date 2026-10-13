# Survey: FPGA NVMe host controllers with no CPU, for a bare Kintex-7 (pcie_7x Gen2 x4)

> Research report produced by an Opus 5.5 research agent (2026-10-14). Items marked **[unverified]** could not be checked.

## 0. Summary

- **Only a few open-source NVMe *host* cores exist.** Most open "NVMe FPGA" projects are the SSD side (device controllers: Cosmos+ OpenSSD, NVMeCHA, OpenExpress, pcileech-nvme).
- **The most relevant open project is LiteNVMe** (enjoy-digital, BSD-2, created Jan 2026, active).
  - It is a full host: enumeration, admin queue, I/O engine, and an optional CPU-less RTL init sequencer.
  - It runs on LitePCIe in RootPort mode. LitePCIe's 7-series PHY (`s7pciephy.py`, which wraps pcie_7x) gained RootPort mode in Jan 2026 and a cfg_mgmt port in Jul 2026.
  - A merged PR (Jul 2026) reports a working 7-series root-port bring-up against a Gen2 x4 NVMe drive: Identify plus block write/read/verify pass.
- **The best design document to learn from is DUNE `pl-nvme`** (Beam Ltd, VHDL, GPL-3). It is a pure-RTL host on the UltraScale Gen3 hard block in root-port mode. It streams writes to 2 SSDs at 4.5 GB/s total and has detailed notes on pitfalls and SSD quirks.
- **Expected ceiling for Gen2 x4 on 7-series is about 1.6–1.7 GB/s sequential write.** Design Gateway measured 1,600–1,700 MB/s on ZC706 Gen2 with a Samsung 990 Pro. IntelliProp got only 750 MB/s write on KC705 Gen2. Design quality and the SSD both matter a lot.
- **I found no off-the-shelf Kintex-7 board with an onboard M.2 host slot.** The usual approach is an HPC FMC carrier (KC705 HPC) plus an FMC-to-M.2 adapter. The adapter carries one 100 MHz clock generator feeding both the SSD (HCSL) and the FPGA (LVDS to GBTCLK). PERST# is driven by an FPGA GPIO, either directly or through a reset IC/jumper.

---

## 1. Open-source NVMe host implementations

### 1.1 LiteNVMe — https://github.com/enjoy-digital/litenvme
- **License / language:** BSD-2-Clause. Migen/LiteX (Python). It can emit a standalone Verilog core with `litenvme_gen <yaml>`.
- **PCIe layer:** LitePCIe `LitePCIeRootPort`.
  - Validated platform: Alibaba Cloud KU3P (XCKU3P, `pcie4_uscale_plus`), Gen3 x4, 256-bit @125 MHz. Earlier development ran on the same board at Gen2 x4, 128-bit.
  - A 7-series RootPort bring-up is reported in PR #1 (https://github.com/enjoy-digital/litenvme/pull/1). It does not name the board **[board unverified]**.
- **Scope:** full host.
  - CFG accessor: BAR sizing and assignment, Command register MEM/BME.
  - MMIO accessor for BAR0 registers, plus a separate MMIO port for doorbells.
  - Root-port cfg_mgmt access, used to set the root port's own DevCtl.MPS and its bridge memory window.
  - Admin path: Identify, Set Features (number of queues), Create IO CQ/SQ.
  - Bring-up options: a VexRiscv firmware path (default) or the CPU-less `LiteNVMeInitSequencer`. The RTL path is hardware-validated on KU3P.
- **How the SSD is served:** a host-memory responder acts as completer on a 256–512 KiB BRAM window. That window holds the SQ/CQ rings, Identify buffer, PRP data buffers and PRP-list pages.
  - The I/O engine (`qsize=64`, `qd=32`) builds SQE + PRP1/PRP2/PRP-list, coalesces SQ doorbells, and reaps CQEs by phase bit.
  - The block streamer frontend is AXI-Stream, with one NVMe command per start. Transfers are limited by the staging size (192 KiB); multi-chunk streaming is listed as future work.
- **Throughput:**
  - Gen3 x4: about 2.69 GB/s read and 2.74 GB/s write (8 KiB, QD32) on a Crucial P310 (QLC, DRAM-less).
  - Gen2 x4, 128-bit (earlier): reads about 1.0–1.15 GB/s with MPS 128; writes about 1.5 GB/s. The authors flag the write figure as possibly cache-acked.
- **Resources (core only, XCKU3P):** 11.6k LUT, 14.4k FF, 133 BRAM including the PCIe IP and CPU; about 108 BRAM with RTL init.
- **Bugs they hit and documented (`doc/ARCHITECTURE.md`, `doc/archive/PROGRESS.md`):**
  - **Root-port MPS defaults to 128 B.** Raising only the SSD's MPS (to 256/512) made reads time out, because the root complex could not accept the larger TLPs. Both ends must be raised; the root's DevCtl is reachable only through the hard-IP cfg_mgmt port. Going to 512 B gave +14.5% on 4 KiB reads.
  - **7-series specific: completion header fields.** On pcie_7x the user logic must fill the CplD Byte Count and Lower Address fields; the UltraScale+ adapter does this automatically. Byte Count = 0 means 4096 bytes remaining, so the SSD waited forever after its SQE fetch: silent admin hang, no CQE, no error bit.
  - **Sub-beat CQE writes.** 16-byte CQEs land at non-beat-aligned offsets (for example ACQ+16). The responder must shift the data to the dword offset and strobe only those bytes. A variable shift simulated correctly but synthesized to no shift; the fix was a constant-shift `Case`.
  - **Completion matching.** Completions are delivered in order on a shared tag controller. A read accessor must consume through the `end` beat, otherwise the next read returns stale data.
  - **CID reuse at ring wrap.** Using CID = SQ slot index caused SC=0x03 (Command ID Conflict) exactly at the first wrap (idx = qsize). A per-slot CID-free gate fixed it; a "torn CQE read" theory was tested and ruled out.
  - **Self-inflicted throughput limits.** A single-beat AXI write path into host memory capped reads at about 460 MB/s, and the engine was accidentally holding only 1 read in flight. They added on-chip duty/stall counters to separate link-bound from core-bound behaviour.
  - **Benchmark caveat.** Write numbers above link rate were write-cache artifacts; measure sustained throughput with large, distinct data.

### 1.2 DUNE pl-nvme (Beam Ltd / Dr Terry Barnaby) — https://github.com/DUNE/pl-nvme
- **License / status:** GPL-3.0, VHDL, last push Aug 2020 (release 1.0.1).
- **PCIe:** Xilinx UltraScale "PCIe Gen3 Integrated Block" in root-port mode, Gen3 x4, 128-bit @250 MHz (the clock is derived from the SSD carrier's 100 MHz refclk). Tested on KCU105 (XCKU040) with the AB17-M2FMC and Opsero OP47 adapters. **Not 7-series**, but the architecture ports directly.
- **Scope:** full pure-RTL host.
  - `NvmeConfig` replays a ROM of TLPs after reset. Command register = 0x0006, BAR0 left at 0, then controller registers, admin queue and IO queues.
  - Alternatively, a host PC can drive the NVMe directly over a debug TLP stream.
- **Architecture:** internal PCIe "packet switch". Address bits [27:24] route requests and Requester ID routes replies. Engines: NvmeQueues, NvmeConfig, NvmeWrite, NvmeRead.
  - `NvmeQueues` stores SQEs in RAM, rings doorbells, and does not store CQEs: it redirects each completion to the requesting engine using the top 8 bits of the CID.
  - The write engine keeps 8×4 KiB buffers, a write queue of 16 entries, and sends Trim/Deallocate interleaved with writes.
- **Measured:** 4,530 MB/s total with 2 SSDs (Samsung 970 Pro or FireCuda 520). Peak write latency was 10.5 ms.
- **Pitfalls and quirks documented:**
  - PCIe payload stuck at 128 B (10% header overhead).
  - The UltraScale RQ needs `last_be` in tuser at the start of the packet.
  - "Some NVMe's do not have the ability to [reset] over their communication interfaces", so the FPGA must drive PERST#. Hold reset at least 100 ms.
  - Error reporting is poor: generic status 6 (internal error) with no asynchronous event detail.
  - Per-drive notes:
    - 970 Pro: MPS 256 B, 512 B LBAs only, about 2,200 MB/s.
    - WD SN750: MPS 512 B but only about 1,024 MB/s sustained.
    - FireCuda 520: falls to 500–1,000 MB/s when writing more than 100 GB to a partly full drive.
  - Trimming the whole region before capture, plus over-provisioning, keeps write latency down.

### 1.3 mcrl/NVMe (Seoul National University) — https://github.com/mcrl/NVMe
- **License / language:** BSD-style (SNU copyright), SystemVerilog. Very active (pushed Jul 2026).
- **Platform:** Bittware 250-SoC (ZU19EG). The SSD-side root port is a Xilinx block design with an AXI bridge (m_axi / s_axi), so this is an AXI-bridge root port rather than raw TLPs. Not 7-series.
- **Scope:** full host in programmable logic. `nvme_bringup.sv` is a microcode sequencer covering enumeration, MPS/MRRS = 256, CC.EN/CSTS, AQA/ASQ/ACQ and IOCQ/IOSQ creation.
- **Notable design: "inverted queue".** There are no SQ/PRP structures in RAM. When the SSD issues an AXI read of the SQ, the FSM synthesizes the SQE on the fly. PRP-list reads are generated arithmetically, and write payload is served from a FIFO. This is a good idea for streaming writes.
- **Measured:** about 2.4 GB/s write and 2.4–2.84 GB/s read (Gen3 x4, Samsung 980 Pro / PM9A1).
- **Pitfalls:**
  - The PRP-list generator wrongly re-served entry 0. The SSD fragments PRP-list reads arbitrarily, so the generator must capture each read's offset.
  - CQEs can arrive back to back.
  - Their claim that "QD>1 gives no bandwidth" (the SSD moving only one command's data) looks like a design or ordering artifact, not SSD behaviour **[treat with skepticism]**.

### 1.4 Other open host-side projects (partial, or CPU/host assisted)

| Project | Notes |
|---|---|
| open-power/snap, Eideticom "NVMe to AXI Bridge" — https://github.com/open-power/snap (`hardware/hdl/nvme`, `hardware/doc/NVMe_Host_Tech_Desc.pdf`) | Apache-2.0, SystemVerilog. Uses Xilinx PCIe root complexes in AXI bridge mode (UltraScale+, 2 SSDs). Hardware SQ/CQ FSMs for I/O; admin entries are written by host software. Partial. |
| DirectNVM — https://github.com/yu-zou/DirectNVM (archived 2023; paper ACM TECS, doi 10.1145/3463911) | Built on Opsero's `fpga-drive-aximm-pcie` plus PetaLinux. Hardware I/O queue modules; Linux does the setup. Partial. |
| FastPath_MP — https://github.com/beehive-lab/FastPath_MP | Apache-2.0. Zynq-7000 with a modified Linux NVMe driver. Partial. |
| shengwenliang/fpga-nvme-controller — https://github.com/shengwenliang/fpga-nvme-controller | Chisel. I/O queue offload through QDMA on Alveo U50; the host CPU initializes the SSD (P2P). Partial. |
| lyftfc/nvme-p2p-driver — https://github.com/lyftfc/nvme-p2p-driver | MIT, Bluespec. SPDK on the host sets up queues in FPGA memory. Partial. |
| krishna-swaroop/axi-nvme — https://github.com/krishna-swaroop/axi-nvme | BSD-2. Planning/verification bootstrap only; it explicitly recommends extending LiteNVMe. |
| zhoulian1988/PCIe-NVMe-Bridge — https://github.com/zhoulian1988/PCIe-NVMe-Bridge | MIT, VHDL. A transparent host↔SSD TLP bridge (endpoint plus root port), claimed on "K7 325T/485T Gen2 x8". Mostly checked-in Vivado IP; useful only as evidence that a K7 root port works with SSDs. |
| antmicro/nvme-verilog-pcie, NVMeCHA, Cosmos+ OpenSSD | Device side or verilog-pcie forks. Not hosts. |
| Chinese CSDN posts ("纯逻辑 NVMe", for example 3,125/2,960 MB/s claims) | Commercial pitches; no source. CSDN blocked fetching **[unverified]**. |

---

## 2. Commercial cores (context)

| Vendor / IP | 7-series? | Architecture | Published numbers |
|---|---|---|---|
| **Design Gateway NVMe-IP** (https://dgway.com/NVMe-IP_X_E.html) | **Yes**: AC701, KC705, VC707, VC709, ZC706. Uses pcie_7x Gen2 x4, 128-bit AXI4-Stream @125 MHz (Gen3 on VC709). | Fully CPU-less. 256 KB BRAM buffer (K7: 66 BRAM tiles, about 3.2k LUT / 4.3k FF). User clock must be ≥ PCIe clock because data must be valid every cycle within a frame. Data moves in 512 B bursts. Commands: Identify, Shutdown, Write, Read, SMART, Secure Erase, Flush. TimeOutSet register. Requires MPSMIN = 4 KB, MDTS ≥ 128 KB, MQES ≥ 7. Revision 3.01 added support for "Memory Write Request not aligned to 128-bit", i.e. CQEs at 16 B offsets. | Gen2 (ZC706): 1,600–1,700 MB/s write and read. Gen3 (KCU105): 3,000 MB/s write / 3,400 MB/s read (990 Pro, 128 GB transfer, all-zero pattern). |
| IntelliProp IPC-NV164-HI | Demo on KC705 Gen2 with Opsero FPGA Drive | CPU-less option (state machine), uses third-party root-complex IP, buffer in BRAM or DDR | KC705 Gen2: **750 MB/s write**, 1,270 MB/s read (Samsung 950 Pro). KCU105: 1.0 GB/s write / 2.0 GB/s read (https://www.fpgadeveloper.com/2016/10/nvme-host-ip-tested-on-fpga-drive.html). |
| MLE NVMe Streamer (https://www.missinglinkelectronics.com/ip-cores/nvme-streamer/) | Brochure lists Versal and ZU+ only | 2019 tech brief: I/O queues in logic, but enumeration, Identify and admin run on a **MicroBlaze bare-metal** CPU. Recommends a 512 KB to multi-GB buffer against SSD latency spikes. | About 3.1 GiB/s at Gen3 x4 |
| Logic Design Solutions NVMe Host IP (Kintex-7 / ZC706 variants) | Claimed | **[details unverified, login wall]** | — |
| iWave NVMe Host IP | Agilex/US+; 7-series not stated | AXI user interface | up to 7 GB/s (Gen4/x8) |

---

## 3. PCIe building blocks for a 7-series root port

### (1) alexforencich/verilog-pcie — https://github.com/alexforencich/verilog-pcie
- MIT. Last push Apr 2024; superseded by Taxi (https://github.com/fpganinja/taxi), whose PCIe support is UltraScale/US+ only.
- **No pcie_7x (PCIE_2_1, Kintex-7) support.** Its "7-series" mention refers to the Virtex-7 **Gen3** block (pcie3_7x), which uses the UltraScale RQ/RC/CQ/CC interface. Kintex-7's pcie_7x has a single TX/RX AXI-Stream (TRN-style) interface, so the `pcie_us_if` shim does not apply.
- **No root-port logic.** cocotbext-pcie hard-core models are explicitly "device only, not root port".
- **Reusable anyway:** the generic TLP-level modules (`pcie_axi_master` = inbound MemRd/MemWr to AXI, i.e. an SSD-facing completer; `dma_if_pcie`, `pcie_tlp_mux/demux/fifo`). You would write your own pcie_7x ↔ generic-TLP shim.
- cocotbext-pcie's Python root-complex/switch/endpoint models could host a home-made NVMe device model for simulation.

### (2) LitePCIe — https://github.com/enjoy-digital/litepcie
- **Yes, root-port mode on 7-series exists, but it is recent and lightly validated.**
  - `S7PCIEPHY(mode="RootPort")` was added 2026-01-20 (PR #158, plus a "fix RootPort compilation" follow-up). It sets `Device_Port_Type = Root_Port_of_PCI_Express_Root_Complex` and class code 0x0600.
  - `LitePCIeRootPort` was added 2026-01-27.
  - PR #170 (2026-07-14) added the configuration-TLP path for the shared-channel S7 PHY and a `with_cfg_mgmt` port. The commit message says this was needed "as LiteNVMe does on Ultrascale+".
  - Other 2026 fixes: user clock derived from width ("x4 at 64-bit requires 250 MHz"), exact BAR sizing, ECRC.
- The only evidence of NVMe working through it is LiteNVMe PR #1 (Gen2 x4 drive, Identify plus write/read/verify). **No 7-series throughput numbers are published.**

### Other blocks
- **Xilinx pcie_7x "Configurator" root-port example (PG054 ch. 5).** Files `cgator*.v` with a ROM of CfgRd/CfgWr TLPs. It up-trains 2.5 to 5.0 GT/s itself. It has **no completion timeout** (it waits forever) and does not throttle RX. Vendor-supplied, so the most trusted starting point for enumeration. There is also an "Endpoint Model Test Bench for Root Port" for simulation.
- **chili-chips-ba/openPCIE** — https://github.com/chili-chips-ba/openPCIE (BSD-3, NLnet-funded, active Oct 2026). Open root-complex RTL around PCIE_2_1 + GTP for **Artix-7 Gen2** (SQRL Acorn CLE-215+). Uses a bare-metal RISC-V driver for enumeration and BAR sizing. Only PIO single-DWORD traffic, tested RC↔FPGA-EP; no NVMe. Builds with Vivado and openXC7. Co-simulation uses wyvernSemi pcievhost (https://github.com/wyvernSemi/pcievhost), a useful open BFM for RC/EP simulation.
- **regymm/pcie_7x** — https://github.com/regymm/pcie_7x. Open PIPE/GT wrapper around PCIE_2_1 with no Vivado IP, endpoint only. Useful to understand clocking and GTP/GTX pipe resets.
- **Xilinx "AXI Memory Mapped to PCI Express" (PG055, axi_pcie) in Root Port mode.** This is what Opsero's KC705/VC707 designs use, with MicroBlaze and Linux. You could drive its ECAM/AXI-Lite from an RTL sequencer and get a CPU-less AXI-bridge host (the mcrl/SNAP style). Its Gen2 x4 throughput is **[unverified]** and it is likely below a raw-TLP design.
- ultraembedded: no PCIe cores (only USB/SATA-type cores).

---

## 4. Comparison table

| Project | License | HDL | Target / PCIe IP | Full host? | CPU at runtime | SQ/PRP/data completer | Data path | Throughput (claimed) | Tested SSDs | Maturity |
|---|---|---|---|---|---|---|---|---|---|---|
| LiteNVMe | BSD-2 | Migen → Verilog | KU3P pcie4_uscale_plus; **7-series via LitePCIe S7 RootPort (Gen2 x4, functional only)** | Yes (enum + admin + IO, optional RTL init) | None with RTL init (otherwise VexRiscv for bring-up only) | BRAM host-memory window 256–512 KiB; QD32, qsize 64; PRP list | 256b@125 MHz (Gen3), 128b (Gen2) | 2.7 GB/s Gen3; Gen2 about 1.0 GB/s read / about 1.5 GB/s write | Crucial P310 | Active 2026, hardware-validated on one board |
| DUNE pl-nvme | GPL-3 | VHDL | KCU105, UltraScale Gen3 hard block RP | Yes (ROM init) | None | BRAM SQ; CQ redirected to engines; 8×4 KiB data buffers; queue 16 | 128b@250 MHz | 4.5 GB/s with 2 SSDs (about 2.2 GB/s each) | 970 Pro, FireCuda 520, SN750 | Released 2020, dormant |
| mcrl/NVMe | BSD-style | SV | ZU19EG, AXI-bridge RP (Gen3 x4) | Yes (HW sequencer) | None | On-the-fly SQE/PRP-list synthesis; 128 KB SRAM + DDR4 | 256b@125 MHz | about 2.4 GB/s write, 2.4–2.84 GB/s read | 980 Pro / PM9A1 | Active 2026, research |
| SNAP / Eideticom | Apache-2 | SV | US+ (Nallatech 250S), AXI-bridge RP | Partial (SW admin) | Host SW | BRAM Tx/Rx buffers, 218-entry IO SQ | 128b | n/p | n/p | Dormant 2021 |
| DirectNVM | n/s | VHDL/Verilog/C | Zynq US+, Opsero AXI-PCIe | Partial (Linux init) | Yes | — | — | paper | — | Archived |
| openPCIE | BSD-3 | SV/Verilog | **Artix-7 PCIE_2_1 RC Gen2** | No (PCIe RC only, PIO) | RISC-V bare-metal | n/a | — | n/a | none (FPGA EP) | Active 2026 |
| DG NVMe-IP | Commercial | encrypted | **7-series pcie_7x Gen2 x4**, US, US+, Versal | Yes | None | 256 KB BRAM | 128b@125 MHz (Gen2) | **1.6–1.7 GB/s Gen2**; 3.0/3.4 GB/s Gen3 | 990 Pro and many | Mature |
| IntelliProp NV164-HI | Commercial | — | KC705 Gen2, KCU105 | Yes | Optional | user BRAM/DDR | — | KC705: 0.75 GB/s write / 1.27 GB/s read | 950 Pro | Mature |
| MLE NVMe Streamer | Commercial | — | ZU+/Versal | Yes | MicroBlaze for admin (2019 brief) | PL RAM/DDR | — | about 3.1 GiB/s Gen3 | — | Mature |

---

## 5. Boards: Kintex-7 with an M.2 slot where the FPGA is host

- **I found no mainstream Kintex-7 board with an onboard M.2 M-key host slot.** Alinx AX7325B (LPC FMC, so x1 only), Alinx AV7K325, PUZHI PZ-K7325T-FH (HPC FMC), HiTech Global HTG-K700, Numato Nereid and Digilent Genesys 2 all need an FMC or other adapter. Several "Artix-7 M.2" boards (Numato Aller, Acorn/NiteFury) are M.2 *endpoint* modules, not hosts.
- **Standard approach:** KC705 HPC FMC plus an adapter. The XC7K325T has **one** PCIe block, so only one SSD at x4.

| Adapter | Refclk | PERST# | Notes |
|---|---|---|---|
| **Opsero FPGA Drive FMC Gen4 (OP063)** — https://docs.opsero.com/op063/datasheet/detailed-description/ | One DSC557-0334 generator per SSD, outputting 100 MHz **HCSL to the SSD** and **LVDS to FMC GBTCLK0/1_M2C**, so FPGA and SSD share a common clock | FPGA output (VADJ), active-high at the FPGA, inverted to 3.3 V active-low by FET circuits | CLKREQ# and PEWAKE# not connected. SSD B powered by a 12 V→3.3 V buck. KC705 support: HPC = SSD1 x4 Gen2, LPC = x1 Gen2 (https://docs.opsero.com/op063/datasheet/compatibility/). Reference designs (MicroBlaze + axi_pcie + Linux): https://github.com/fpgadeveloper/fpga-drive-aximm-pcie |
| **Design Gateway AB17-M2FMC** — https://dgway.com/products/IP/ABseries/AB17-M2FMC-MAN-E.pdf | IDT5V41236 generator: 100 MHz, no SSC by default (JP1–JP3), HCSL to SSDs and LVDS to FMC, same clock to both | J1 jumper: FMC reset routed directly to SSD PERST, or a TPS3808 reset IC (100 ms delay) on SSD/FMC, plus a manual button. pl-nvme used 1-2 & 3-4 "so the FPGA generates NVMe resets". | Warns FMC 12 V (2 A) is marginal for 2 SSDs; auxiliary ATX 4-pin input provided |
| Alinx FH1402 (2× M.2 x4, HPC) | **[clock/PERST unverified]** | **[unverified]** | — |
| circuitvalley FMC_NVMe_PCIe_Drive_Adapter (https://github.com/circuitvalley/FMC_NVMe_PCIe_Drive_Adapter) | open hardware, 2× x4 | details not checked | — |

- **Common wiring pattern:** one low-jitter 100 MHz clock (no SSC), a fanout buffer, HCSL to the M.2 slot and LVDS/HCSL to the FPGA MGTREFCLK of the GTX quad. This is a common-clock architecture; consumer SSDs generally do not support SRIS **[general knowledge]**.
  - PERST# comes from FPGA logic, open-drain or inverted to 3.3 V.
  - Hold PERST# at least 100 ms after 3.3 V is stable, with refclk running before release.
  - CLKREQ# is tied low or left unconnected (openPCIE's backplane notes "clock generator requires CLKREQ to be pulled to GND").

---

## 6. What we should reuse / what to design ourselves / top-10 pitfalls

### Reuse
1. **Xilinx pcie_7x in Root Port mode** (Gen2 x4, 128-bit @125 MHz, or 64-bit @250 MHz).
   - Set Capability MPS to 512 (1024 is allowed for x1–x4 Gen2).
   - Use the "High" performance receive-buffer level.
   - Start from the **Configurator (cgator) example** for link-up, Gen1→Gen2 up-train and the ROM-driven CfgWr sequence, and its Endpoint-Model test bench.
2. **Study LiteNVMe** (`hostmem.py`, `io_engine.py`, `init.py`, the LitePCIe `s7pciephy.py` RootPort plus cfg_mgmt).
   - Either generate its Verilog core targeting S7PCIEPHY, or port its proven init order and fixes.
   - It is BSD-2, so it can be used in a closed design.
3. **Borrow pl-nvme's write strategy:** ROM-driven init, multiple 4 KiB-or-larger outstanding writes, Trim of the target region before capture, and a peak-latency register.
4. **Borrow mcrl's "synthesize SQE / PRP-list on the fly" idea** for a streaming writer, so SQ and PRP lists never live in RAM.
5. **For simulation,** use verilog-pcie TLP modules and cocotbext-pcie, or pcievhost, with your own small NVMe device model. The model should post CQEs non-atomically, track CIDs, and split MemRd requests.

### Design ourselves
- A pcie_7x TX arbiter with completer priority: CplD to the SSD must flow at line rate, with zero-bubble 128-bit beats.
- An SSD-facing completer:
  - Handles MemRd of up to MRRS (typically 512 B).
  - Splits CplDs at MPS and RCB boundaries.
  - Sets Byte Count and Lower Address correctly.
  - Is fully pipelined across many outstanding tags.
- A streaming write data path: input FIFO, a ring of N × (128 KiB to MDTS) buffers, and PRP synthesis.
- A minimal state-machine admin path: CC/AQA/ASQ/ACQ, Identify (for LBA format and MDTS), Create IO CQ then IO SQ, optional Set Features, DSM/Trim, Flush and Shutdown on stop.
- CQ reaping with the phase bit, CQ-head doorbell, and a per-CID free gate.
- Timeouts everywhere: MMIO completions, CSTS.RDY using CAP.TO, I/O completion.

### Top 10 pitfalls
1. **Root port's own config space.** On pcie_7x, set the root port's Command.BME (bit 2) through the **cfg_mgmt** port. Otherwise every SSD DMA (SQ fetch, data read, CQE write) gets UR (PG054 Table 3-25). The root port does no BAR or Base/Limit filtering: all inbound TLPs reach your logic, so you decode addresses.
2. **MPS/MRRS mismatch.** The root port defaults to 128 B MPS. Program the root's DevCtl.MPS via cfg_mgmt **and** the SSD's DevCtl (MPS = min of both DevCaps; MRRS 512 B or more). Raising one side only causes silent timeouts (LiteNVMe). 128 B MPS costs about 10–15%.
3. **Completion formatting on raw-TLP 7-series.** You must fill Byte Count, Lower Address, Completer and Requester ID and the tag yourself. A wrong Byte Count (0 = 4096) caused a silent admin hang on LiteNVMe's 7-series bring-up. Split large reads into multiple CplDs no larger than MPS.
4. **Sub-beat and unaligned writes from the SSD.** CQEs are 16 B, so they land mid-beat in 128-bit RAM. Honor byte enables and offsets (DG revision 3.01; LiteNVMe bug). Also watch out for synthesis of variable shifts.
5. **No completion timeout in the core or the Configurator.** pcie_7x does not time out non-posted requests (PG054 "Completion Timeouts"). Add timeouts to every CfgRd and MMIO read. Handle CRS (Configuration Request Retry Status) on early config reads after PERST#: retry until the SSD is ready.
6. **Link speed and training.** The root port trains at Gen1 first. Up-train to Gen2 the way cgator does (the RP_AUTO_SPD rules apply). Check the link really trained at x4 Gen2 (status registers / `pl_*` signals). Lane mapping and polarity on the FMC GTX quad must match the adapter.
7. **Reset and clock sequencing.**
   - The FPGA must own PERST#: some SSDs cannot otherwise be reset (pl-nvme).
   - Hold it at least 100 ms after power-good, with refclk running before release.
   - The FPGA GTX refclk must be the **same** 100 MHz source as the SSD's (common clock, no SSC unless both sides support it).
   - The M.2 3.3 V supply can draw 2.5–3 A. FMC 12 V is marginal; check the adapter's power path.
8. **Queue protocol details.**
   - Phase bit flips at wrap.
   - Ring the CQ-head doorbell promptly, including the wrap to 0.
   - Never reuse a CID until its CQE is reaped: per-CID free gate, not just an in-flight count (SC=0x03 at idx = qsize in LiteNVMe).
   - CQEs can arrive back to back; buffer the full CQE.
   - Doorbell stride from CAP.DSTRD; wait for CSTS.RDY using CAP.TO; CC.IOSQES=6, CC.IOCQES=4, CC.MPS matching 4 KiB.
9. **PRP rules.**
   - Memory page = 4 KiB. PRP1 may carry an offset; PRP2 and PRP-list entries must be page-aligned.
   - Transfers over 2 pages need a PRP list, which the SSD fetches in arbitrary chunks (mcrl bug).
   - Respect MDTS (DG requires 128 KB or more). Use large commands (64 KiB to MDTS) with several in flight to amortize per-command overhead.
10. **SSD behaviour dominates sustained writes.**
    - SLC-cache exhaustion, QLC and DRAM-less (HMB-requesting) drives collapse to 0.5–1 GB/s after the cache fills. Examples: FireCuda 520 partly full; SN750 about 1 GB/s; LiteNVMe's QLC P310 shows run-to-run variance.
    - Write latency spikes are around 10 ms (pl-nvme measured 10.5 ms).
    - Choose a TLC/MLC drive with sustained write above 1.7 GB/s, Trim the region first, add a heatsink (thermal throttling), and size the buffer for tens of ms (or DDR for longer).
    - Benchmark with large distinct data, not cached or zero patterns.

**Throughput reality check for our target.** Gen2 x4 is 2.0 GB/s raw after 8b/10b. With 256–512 B completions and DLLP overhead the practical ceiling is about 1.7–1.8 GB/s. The best published Gen2 figure is DG's 1.6–1.7 GB/s. A 128-bit @125 MHz datapath equals link rate, so the completer must sustain 1 beat per clock with no bubbles. Note: pcie_7x streaming mode (`tx_str`) forbids mid-packet gaps; the default store-and-forward mode allows throttling.

---

## 7. Could not verify
- Which 7-series board LiteNVMe's PR #1 used, and any 7-series throughput.
- Throughput of the Xilinx axi_pcie root-port bridge on 7-series.
- Alinx FH1402 clock/PERST wiring.
- LDS and iWave 7-series details.
- Chinese CSDN pure-logic NVMe claims (CSDN blocked fetching).
- MLE NVMe Streamer's current 7-series support (the brochure lists only US+/Versal).
- The exact cgator Gen2 up-train mechanism (PG054 says it up-trains; check `cgator_controller.v`).
