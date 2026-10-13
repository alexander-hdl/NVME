# Minimal NVMe host for a CPU-less FPGA streaming writer (PCIe Gen2 x4, Kintex-7): implementation brief

> Research report produced by an Opus 5.5 research agent (2026-10-14). Facts checked against
> NVM Express Base Specification 1.4 and the Linux `drivers/nvme/host` sources. Items marked
> **[UNVERIFIED]** are engineering estimates / secondary sources and must be measured on hardware.

---

## 0. TL;DR architecture

- **Your role.** The FPGA is the Root Port (RP) and the host memory. The SSD is the bus master: it fetches SQEs, PRP lists and data with Memory Reads (MRd), and posts CQEs (and Identify data) with Memory Writes (MWr).
- **What the FPGA has to do:**
  - issue a handful of config and MMIO TLPs;
  - answer every MRd quickly and in address order per tag;
  - accept MWr.
- **The SSD sets the bandwidth.** It is the requester, so its outstanding-read window sets the throughput. Your job is low completion latency and no back-pressure.
- **Settings to use:**
  - one I/O SQ/CQ pair (QID 1), 4 KiB MPS (CC.MPS=0), PRP data pointers, interrupts off (IEN=0), polled or snooped CQ;
  - 128 KiB Write commands (fits any MDTS ≥ 5), QD 8 to 32, each command pointing at a 128 KiB ring slot;
  - the PRP list can be synthesized on the fly from the slot address. No RAM is needed for it.
- **Expected throughput:** PCIe Gen2 x4 gives about 1.70 GB/s theoretical payload at MPS=128 and about 1.82 GB/s at MPS=256. In practice expect about 1.6 to 1.75 GB/s.
- **Before power-off:** always do the orderly shutdown (CC.SHN=01b, wait for CSTS.SHST=10b).

---

## 1. Controller registers (BAR0), NVMe 1.4 §3.1

The host may use native width or aligned 32-bit accesses (§3.1 preamble). Linux writes ASQ and ACQ as two 32-bit writes, low half first (`lo_hi_writeq`). An access that spans two registers is not allowed.

| Offset | Name | Width | Notes |
|---|---|---|---|
| 00h | CAP | 64 | RO |
| 08h | VS | 32 | MJR[31:16], MNR[15:8], TER[7:0]; 1.4 reads 0x00010400 |
| 0Ch | INTMS | 32 | Interrupt mask set (only when MSI-X is not used) |
| 10h | INTMC | 32 | Interrupt mask clear |
| 14h | CC | 32 | |
| 1Ch | CSTS | 32 | |
| 20h | NSSR | 32 | Optional; writing 0x4E564D65 ("NVMe") triggers an NVM subsystem reset |
| 24h | AQA | 32 | |
| 28h | ASQ | 64 | |
| 30h | ACQ | 64 | |
| 38h to 5Ch | CMBLOC, CMBSZ, BPINFO, … | | Not needed |
| 1000h + (2y)·(4<<DSTRD) | SQyTDBL | 32 | Bits 15:0 = new tail |
| 1000h + (2y+1)·(4<<DSTRD) | CQyHDBL | 32 | Bits 15:0 = new head |

With DSTRD=0: SQ0TDBL=0x1000, CQ0HDBL=0x1004, SQ1TDBL=0x1008, CQ1HDBL=0x100C.

### CAP (Fig. 69)

| Bits | Field | Meaning |
|---|---|---|
| 63:58 | rsvd | |
| 57 | CMBS | Controller Memory Buffer supported |
| 56 | PMRS | Persistent Memory Region supported |
| 55:52 | MPSMAX | Max page = 2^(12+MPSMAX) |
| 51:48 | MPSMIN | Min page = 2^(12+MPSMIN). Must be 0 for a 4 KiB host page. |
| 47:46 | rsvd | |
| 45 | BPS | Boot partitions |
| 44:37 | CSS | Bit 37 = NVM command set; bit 44 = admin only |
| 36 | NSSRS | NVM subsystem reset supported |
| 35:32 | DSTRD | Doorbell stride = 2^(2+DSTRD) bytes |
| 31:24 | TO | Worst-case wait for CSTS.RDY to toggle after CC.EN toggles, in **500 ms units** |
| 23:19 | rsvd | |
| 18:17 | AMS | Bit 17 = WRR+urgent, bit 18 = vendor. Round robin is always supported. |
| 16 | CQR | Contiguous queues required (then PC must be 1) |
| 15:0 | MQES | Max entries per I/O queue, **0's based**, minimum 1 (two entries) |

Linux re-reads CAP after the first CC write ("CAP value may change after initial CC write"). Newer controllers can also report CRTO (Base 2.0). Linux uses the larger of CRTO and CAP.TO.

### CC (Fig. 78)

| Bits | Field | Value to write |
|---|---|---|
| 31:24 | rsvd (bit 24 = CRIME in 2.0) | 0 |
| 23:20 | IOCQES | 4 (16 B) |
| 19:16 | IOSQES | 6 (64 B) |
| 15:14 | SHN | 00 = none, 01 = normal, 10 = abrupt |
| 13:11 | AMS | 000 (round robin) |
| 10:7 | MPS | 0 (4 KiB). Change only while EN=0. |
| 6:4 | CSS | 000 (NVM). Change only while EN=0. |
| 3:1 | rsvd | 0 |
| 0 | EN | 0 → 1 |

Values: CC = **0x00460000**, then **0x00460001**. For shutdown write **0x00464001**.

Rules:
- AQA, ASQ and ACQ may only be modified while EN=0.
- Setting EN to 1 while RDY=1, or clearing it while RDY=0, is undefined.
- Creating I/O queues before IOSQES/IOCQES are set fails with Invalid Queue Size.

### CSTS (Fig. 79)

| Bit | Field |
|---|---|
| 5 | PP (processing paused) |
| 4 | NSSRO |
| 3:2 | SHST: 00 = normal, 01 = shutdown in progress, **10 = shutdown complete** |
| 1 | CFS (controller fatal) |
| 0 | RDY |

Host software "shall wait a minimum of CAP.TO" for RDY. RDY may be 1 before the namespaces are ready.

### AQA, ASQ, ACQ (Figs. 81 to 83)

- **AQA:** ACQS[27:16] and ASQS[11:0], both 0's based, 2 to 4096 entries. Writing 0 is undefined. Example: 0x001F001F gives 32 entries each.
- **ASQ / ACQ:** bits 63:12 are the base address. It must be page aligned (4 KiB) and physically contiguous.

### Doorbells (§3.1.24 and §3.1.25)

- 32-bit; only bits 15:0 are used.
- The SQ tail doorbell value is the index of the next free slot.
- The CQ head doorbell value is the index of the next entry you have not yet consumed.

---

## 2. Initialization sequence (NVMe 1.4 §7.6.1), mapped to a minimal hardware host

The spec's 11 steps, condensed:
1. Configure PCI/PCIe registers.
2. Wait for CSTS.RDY=0.
3. Set AQA, ASQ, ACQ.
4. Set CC.AMS, CC.MPS, CC.CSS.
5. Set CC.EN=1.
6. Wait for CSTS.RDY=1.
7. Identify Controller, then Identify each namespace.
8. Set Features, Number of Queues.
9. Create I/O CQs.
10. Create I/O SQs.
11. Optionally, AER commands (skip them).

How to implement it in hardware:

1. **Read CAP.** Check MPSMIN==0 and CSS bit 37==1. Latch MQES, TO and DSTRD.
2. **Reset the controller if it is enabled.** If CC.EN==1, write CC.EN=0. Then poll CSTS.RDY==0, with timeout TO×500 ms. Linux always does this, and waits `(TO+1)/2` seconds. Some devices need a 2300 ms delay before polling (`NVME_QUIRK_DELAY_BEFORE_CHK_RDY`, `NVME_QUIRK_DELAY_AMOUNT=2300`).
3. **Zero the admin CQ, and later the I/O CQ.** §4.6 Fig. 123 requires every Phase Tag to be 0 before CC.EN=1.
4. **Write AQA, ASQ and ACQ.**
5. **Write CC.** First write CC=0x00460000 (EN=0), then CC=0x00460001. This is the Linux pattern: configure, re-read CAP, then enable.
6. **Wait for ready.** Poll CSTS until RDY==1, with timeout TO×500 ms. If CFS==1, abort.
7. **Identify Controller.** Opcode 06h, CNS=01h, NSID=0. The 4 KiB buffer must be page aligned with PRP2=0 (§5.15: the Identify data pointer must not be a PRP list). Fields to read:

| Byte(s) | Field | Use |
|---|---|---|
| 77 | **MDTS** | Max transfer = 2^MDTS × (CAP.MPSMIN page); 0 = unlimited. Example: 5 → 128 KiB. |
| 88–91 | RTD3E | Typical shutdown time, in µs |
| 266–267 / 268–269 | WCTEMP / CCTEMP | Temperature thresholds, in Kelvin |
| 272–275 / 276–279 | HMPRE / HMMIN | Host Memory Buffer request (DRAM-less drives). You may simply not provide an HMB. |
| 512 | **SQES** | Must be 0x66 |
| 513 | **CQES** | Must be 0x44 |
| 514–515 | MAXCMD | |
| 516–519 | **NN** | Number of namespaces |
| 520–521 | ONCS | |
| 525 | **VWC** | Bit 0 = volatile write cache present. Bits 2:1 = Flush with NSID FFFFFFFFh support. |
| 536–539 | SGLS | Optional; ignore it and use PRPs |

8. **Identify Namespace.** CNS=00h, NSID=1. Fields to read:
   - **NSZE** at bytes 0–7 and **NCAP** at bytes 8–15. These are in LBAs.
   - **NLBAF** at byte 25 (0's based).
   - **FLBAS** at byte 26. Bits 3:0 select the format; bit 4 = extended metadata.
   - **LBAF[n]** at 128 + 4n: MS in bits 15:0, **LBADS** in bits 23:16 (2^n bytes; 9 = 512 B, 12 = 4 KiB), RP in bits 25:24.
   - Check that MS=0.
   - Optional: NPWG at 64, NPWA at 66, NOWS at 72 (preferred write granularity, alignment and optimal size). Use them to align SLBA and command size.
   - Most consumer drives ship formatted 512 B (512e). Many also offer a 4 KiB LBAF that can be selected with Format NVM.

9. **Set Features, Number of Queues.** Opcode 09h, CDW10 = 0x00000007 (FID 07h), CDW11 = 0x00000000 (NSQR=0, NCQR=0, meaning one of each, 0's based). The result is in completion DW0: NCQA[31:16], NSQA[15:0]. The spec says Create I/O CQ/SQ QIDs "shall not exceed the value reported in the Number of Queues feature", so issue this command. Linux always does. It must come before any I/O queue is created (otherwise Command Sequence Error).

10. **Create I/O CQ** (opcode 05h), then **Create I/O SQ** (opcode 01h). See §10 for the dword layouts.

11. **Optional:** Get Features or Set Features for the Volatile Write Cache (FID 06h), covered below.

### Volatile Write Cache (FID 06h, §5.21.1.6)

- CDW11 bit 0 is WCE.
- If VWC is present it is normally enabled by default. Leave it enabled: disabling it on consumer drives usually collapses write throughput **[UNVERIFIED, drive dependent]**.
- What matters with the cache on is that data acknowledged by a CQE may still be volatile. So use either:
  - **Flush** (NVM opcode **00h**, NSID=1 or FFFFFFFFh if VWC bits 2:1 = 11b; §6.8), or
  - the **shutdown** sequence before power is removed, or
  - FUA=1 per command, which is slow.

  Consumer drives generally have no power-loss protection.

---

## 3. Submission Queue Entry (64 B, §4.2 Figs. 104/105) and the Write command (§6.15)

### CDW0

| Bits | Field |
|---|---|
| 31:16 | CID (unique among this SQ's outstanding commands) |
| 15:14 | PSDT (00 = PRP) |
| 13:10 | rsvd |
| 9:8 | FUSE (00) |
| 7:0 | OPC |

### SQE layout

| Bytes | DW | Field |
|---|---|---|
| 03:00 | 0 | CDW0 |
| 07:04 | 1 | NSID (FFFFFFFFh = broadcast, only where the command allows it) |
| 15:08 | 2–3 | Reserved |
| 23:16 | 4–5 | MPTR (metadata pointer; 0) |
| 31:24 | 6–7 | **PRP1** |
| 39:32 | 8–9 | **PRP2** |
| 43:40 | 10 | CDW10 |
| 47:44 | 11 | CDW11 |
| 51:48 | 12 | CDW12 |
| 55:52 | 13 | CDW13 |
| 59:56 | 14 | CDW14 |
| 63:60 | 15 | CDW15 |

### Write (NVM opcode 01h)

| DW | Bits | Field |
|---|---|---|
| CDW10 | 31:0 | SLBA[31:0] |
| CDW11 | 31:0 | SLBA[63:32] |
| CDW12 | 31 | LR (limited retry) |
| | 30 | **FUA** |
| | 29:26 | PRINFO |
| | 25:24 | rsvd |
| | 23:20 | DTYPE |
| | 19:16 | rsvd |
| | 15:0 | **NLB, 0's based** |
| CDW13 | 31:16 | DSPEC |
| | 7:0 | DSM: bit 7 incompressible, **bit 6 sequential request**, bits 5:4 access latency, bits 3:0 access frequency |
| CDW14 | | ILBRT (protection information only) |
| CDW15 | | LBATM[31:16], LBAT[15:0] |

Other NVM opcodes: Flush 00h, Write 01h, Read 02h, Dataset Management 09h.

### Example Write SQE: 256 × 512 B LBAs (128 KiB) at SLBA 0x1000, CID 5, ring slot 1

Assumptions: ring base 0x8000_0000 with 128 KiB slots, so slot 1 = 0x8002_0000. PRP lists sit at 0x9000_0000 + slot×0x100.

| Bytes | DW | Value | Meaning |
|---|---|---|---|
| 03:00 | 0 | 0x0005_0001 | CID=5, PSDT=00, FUSE=00, OPC=01h |
| 07:04 | 1 | 0x0000_0001 | NSID=1 |
| 11:08 | 2 | 0 | |
| 15:12 | 3 | 0 | |
| 19:16 | 4 | 0 | MPTR lo |
| 23:20 | 5 | 0 | MPTR hi |
| 27:24 | 6 | 0x8002_0000 | PRP1 lo (page 0 of the slot, offset 0) |
| 31:28 | 7 | 0x0000_0000 | PRP1 hi |
| 35:32 | 8 | 0x9000_0100 | PRP2 lo = PRP **list** pointer (qword aligned; may carry an offset) |
| 39:36 | 9 | 0x0000_0000 | PRP2 hi |
| 43:40 | 10 | 0x0000_1000 | SLBA lo |
| 47:44 | 11 | 0x0000_0000 | SLBA hi |
| 51:48 | 12 | 0x0000_00FF | NLB=255 (256 blocks), FUA=0, LR=0 |
| 55:52 | 13 | 0x0000_0000 | Or 0x40 for the "sequential" DSM hint |
| 59:56 | 14 | 0 | |
| 63:60 | 15 | 0 | |

With a 4 KiB LBA format the same 128 KiB is NLB=31 (0x1F), and SLBA is in 4 KiB units.

Little-endian memory image (byte 0 first):
```
00: 01 00 05 00 01 00 00 00 00 00 00 00 00 00 00 00
10: 00 00 00 00 00 00 00 00 00 00 02 80 00 00 00 00
20: 00 01 00 90 00 00 00 00 00 10 00 00 00 00 00 00
30: FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### Example PRP list at 0x9000_0100: 31 entries, 248 B, ends at 0x...01F7, inside one 4 KiB page

```
entry  0: 0x00000000_80021000
entry  1: 0x00000000_80022000
 ...
entry  i: 0x80020000 + (i+1)*0x1000
 ...
entry 30: 0x00000000_8003F000
```

### Completion Queue Entry (16 B, §4.6, Figs. 121 to 124)

| DW | Bits | Field |
|---|---|---|
| DW0 | | Command specific (for example Set Features Number of Queues returns NSQA/NCQA) |
| DW1 | | Reserved |
| DW2 | 31:16 | SQID |
| | 15:0 | SQHD (SQ head pointer) |
| DW3 | 31 | DNR |
| | 30 | M |
| | 29:28 | CRD |
| | 27:25 | **SCT** |
| | 24:17 | **SC** |
| | 16 | **P (phase tag)** |
| | 15:0 | CID |

Success means DW3[31:17] == 0.

Example CQE for the command above, first pass through the CQ, SQ head now 6:
`DW0=0x00000000 DW1=0x00000000 DW2=0x00010006 DW3=0x00010005`
(P=1, status 0, CID 5).

Useful status codes:
- **SCT=0:** 00 success, 01 invalid opcode, 02 invalid field, 03 CID conflict, 04 data transfer error, 06 internal error, 0B invalid namespace/format, 0C command sequence error, **13h PRP offset invalid**, 80h LBA out of range, 81h capacity exceeded, 82h namespace not ready (retry if DNR=0).
- **SCT=1, Create queue:** 00 CQ invalid, 01 invalid QID, 02 invalid queue size, 08 invalid interrupt vector.
- **SCT=2:** 80h write fault.

### Phase-tag polling

- The host zeroes the CQ before EN.
- On the first pass the controller writes P=1; after it wraps it writes P=0, and so on (§4.6 Fig. 123).
- Keep `head` and `expected_phase` (start at 1).
- An entry is new when `CQE[head].P == expected_phase`.
- Consume it, then `head++`. If `head==qsize`, set `head=0` and `expected_phase ^= 1`.
- Then write CQyHDBL = head. You may batch several entries into one doorbell write.
- If a CQE is built from several writes, P is written last.
- SQHD tells you which SQ slots are free again.

**Hardware shortcut:** the CQ can be a decoded address window instead of a polled RAM. Parse every MWr that hits the CQ range as a stream of 16 B entries. One MWr may carry several CQEs. You still need head tracking and the CQ head doorbell; the phase check then works as a sanity check.

---

## 4. PRP rules (§4.3, Figs. 105/107/108/109)

- **PRP1** may carry a page offset, which must be dword aligned (bits 1:0 = 0).
- All other PRP entries, and every entry inside a PRP list, must have offset 0. Otherwise the controller should return PRP Offset Invalid.
- **PRP2** rules (Fig. 105):
  - (a) reserved if the transfer does not cross a page boundary;
  - (b) the **second page address** if the transfer crosses exactly one boundary, for example offset 0 and length > 4 KiB but ≤ 8 KiB;
  - (c) a **PRP list pointer** if it crosses more than one boundary.
- The PRP list pointer must be qword aligned and **may have a non-zero offset**. A list occupies at most one page, and its entries are packed starting at entry 0.
- If more entries are needed, the **last entry of the page** points to the next list page, which must be page aligned.

### Entry count for page-aligned PRP1 and 4 KiB MPS

```
pages = ceil((PRP1_offset + len) / 4096)
pages == 1        -> PRP2 unused (0)
pages == 2        -> PRP2 = second page address
pages  > 2        -> PRP2 = list pointer, list entries = pages - 1
                     (one list page holds 512 entries; if pages-1 > 512 (minus
                      the starting offset), entry 511 chains to the next page)
```

For **128 KiB**: pages = 32, so PRP1 plus **31 list entries (248 B)**. For 256 KiB: 63 entries (504 B). For 2 MiB (MDTS=9): 511 entries, which exactly fills a page started at offset 0.

### Who reads what over PCIe

For a Write command, everything moves SSD ← host:

1. Host MWr to the SQ tail doorbell.
2. SSD MRd of the SQE (64 B, or several SQEs in one read).
3. SSD MRd of the PRP list. It may read only part of it, or read it in chunks.
4. SSD MRd of the data pages, at most MRRS bytes each, with many outstanding at once.
5. Host returns CplD for each read.
6. SSD MWr of the 16 B CQE into the CQ.
7. MSI/INTx would follow, but it is disabled here.

So the SQ, the PRP lists and the data ring must all sit in your memory address space, behind your completer. Any address the SSD reads that you do not decode must get a UR completion (status 001b). Otherwise the SSD hits a completion timeout and may set CFS.

### SGL vs PRP

- PRPs are mandatory for the admin queue over PCIe (§4.2), and every PCIe controller supports PRPs for I/O.
- SGL support is optional (Identify SGLS). Use PRP (PSDT=00).

### Hardware trick

The PRP list for slot k is a pure function: `entry[i] = slot_base(k) + (i+1)*4096`. Decode a "PRP window", for example LIST_BASE + k×256, and generate the completion data arithmetically. No list RAM is needed. You can synthesize SQEs from a small per-slot register (SLBA, CID) in the same way.

---

## 5. Doorbells and queues (§4.1)

- **Empty:** head == tail. **Full:** head == tail+1 modulo size. A queue of size N holds at most N−1 commands.
- **Size:** QSIZE is 0's based, minimum 2 slots, maximum min(64 Ki, CAP.MQES+1). The admin queues are at most 4 Ki.
- **Memory:** SQ memory = 64 B × entries; CQ memory = 16 B × entries. With PC=1 they are physically contiguous and **page aligned** (CC.MPS). The admin queues are always contiguous.
- **Doorbell writes:** one 32-bit MWr (FirstBE=0xF, LastBE=0). A single tail write can submit several SQEs. Coalescing is allowed and slightly helpful, but there is no rule against one doorbell per command; at roughly 24 B on the link each one is negligible.
- **Never:**
  - write a tail or head value outside [0, qsize−1];
  - move the CQ head past entries that are not yet posted;
  - overfill the SQ.

  Doing so raises "Invalid Doorbell Write Value", and the controller may stop processing that queue (§4.1).
- **Do not modify an SQE** after submitting it until the controller has consumed it (SQHD has moved past it).
- **Keep the data buffer valid** until the CQE for that command arrives.
- **If the CQ is full**, the controller stops posting and may stop fetching. Write the CQ head doorbell promptly.

---

## 6. How real SSD controllers issue reads, and how you complete them

Facts from the PCIe specification:
- A requester's MRd length is ≤ **MRRS** in its Device Control register (PCIe capability + 0x08, bits 14:12).
  - Encoding for both MRRS and MPS: 000=128, 001=256, 010=512 (the MRRS reset default), 011=1024, 100=2048, 101=4096.
- The MPS reset default is 000 (128 B). MPS lives in bits 7:5 of the same register.
- **Tags:** 5-bit tags (32 outstanding) by default. With **Extended Tag Field Enable** (Device Control bit 8; supported if Device Capabilities bit 5 = ETFS) a device may use 8-bit tags (256). The NVMe-over-PCIe transport spec references ETFS/ETE.
- **Completion splitting:**
  - Each CplD payload is ≤ MPS.
  - A read may be split into several CplDs, which must be returned in **increasing address order** for that tag. Completions for different tags may be reordered.
  - Split points must fall on naturally aligned **RCB** boundaries (64 B or 128 B; the Link Control bit 3 RCB bit tells the endpoint which value applies).
  - The first CplD may run from the start address up to the first boundary.
  - Safe rule: split only at MPS-aligned addresses (256 is a multiple of both 64 and 128).
- **CplD header (3 DW):**
  - DW0: Fmt/Type = 0x4A, Length in DW. Copy TC and Attr from the request.
  - DW1: Completer ID[31:16], Status[15:13]=000, BCM=0, **Byte Count[11:0]** = bytes remaining *including this CplD* (0 means 4096).
  - DW2: Requester ID[31:16], Tag[15:8], **Lower Address[6:0]** = low 7 bits of the address of the first byte in *this* CplD.
- **Example:** MRd 512 B @ 0x8002_0000 with MPS=256 returns:
  - CplD #1: Len=64 DW, BC=512, LA=0x00;
  - CplD #2: Len=64 DW, BC=256, LA=0x00.

What drives actually do **[UNVERIFIED for specific controllers; measure with ILA/analyzer]**:
- Consumer controllers (Phison E12/E16/E18, SMI SM2262/2264, Samsung Phoenix/Elpis, WD in-house, Marvell) honour whatever MRRS you program.
- On PCs, firmware usually leaves MRRS at 512 B and MPS at 128 or 256 B, and these drives still reach 3.5 to 7 GB/s. So they keep many reads in flight: tens of tags, using extended tags at Gen4.
- They fetch data for several commands concurrently, and data order across commands is not guaranteed.
- Device Capabilities MPSS on consumer SSDs is commonly 256 B, sometimes 512 B. Read it rather than assume.

### Recommendations

- Program **MPS = min(SSD MPSS, RP capability)** on *both* the SSD and your RP. 256 B is typical.
- Set **MRRS to 512 B or more**. 4096 B gives fewer MRd TLPs, but your completer must then split each read into 16 CplDs.
- Set **ETE=1** if ETFS=1.
- **Never throttle non-posted requests.** A cautionary data point: MLE found the Xilinx AXI-PCIe bridge (PG194) default `C_M_AXI_NUM_READQ=2` (2 NP credits) limited a 12-SSD RAID to about 2.7 GB/s. Raising it to 8 gave about 8.6 GB/s.
  - On pcie_7x, keep `rx_np_ok` asserted.
  - Keep `m_axis_rx_tready` high.
  - Make sure the RP advertises enough NP header credits **[check PG054 for credit configuration]**.
- **Completion timeout:** the SSD's Device Control 2 bits 3:0 default to a 50 µs to 50 ms range, and the spec says it should not expire in under 10 ms. A DDR3-backed completer at about 0.3 to 2 µs is far inside that.
  - The real risk is logic that *holds* a completion waiting for data.
  - Rule: **only submit a command once all 128 KiB of its slot are written into the ring.** Never stall CplDs on the producer.

---

## 7. Throughput analysis

### Link

- Gen2 x4 = 4 × 5 GT/s = 20 Gb/s raw. 8b/10b leaves 16 Gb/s = **2.0 GB/s per direction**.
- Per-TLP link overhead for a CplD (3DW header, no ECRC) = STP 1 + seq 2 + header 12 + LCRC 4 + END 1 = **20 B**. Add 4 B with ECRC.
- On x4, TLPs are already 4-symbol multiples, so there is no padding loss.

| MPS | TLP efficiency | Payload GB/s, TLP only | With our Ack/UpdateFC DLLPs (~8 B per serviced MRd) + SKP (~0.3%), MRRS=512 | Same, MRRS=4096 |
|---|---|---|---|---|
| 128 | 128/148 = 86.5% | 1.730 | **≈1.70** | ≈1.72 |
| 256 | 256/276 = 92.75% | 1.855 | **≈1.82** | ≈1.85 |
| 512 | 512/532 = 96.2% | 1.925 | ≈1.89 | ≈1.92 |

The SSD's MRds and its Acks for our CplDs travel in the other direction and do not consume your transmit bandwidth. MRd traffic is about 20 B per 512 B, so about 70 MB/s on the receive direction. SQE and PRP-list fetch is about 0.24% per 128 KiB command.

**Realistic target: 1.6 to 1.75 GB/s at MPS=256.** Gen2 x4 SSDs in PCs typically show about 1.6 to 1.7 GB/s **[UNVERIFIED]**.

### Kintex-7 user bus (pcie_7x x4 Gen2)

- 64-bit @ 250 MHz = 16 Gb/s = 2.0 GB/s, exactly the link line rate. The 128-bit @ 125 MHz option is the same.
- The 7-series transmit interface does not straddle TLPs, so each TLP starts on a new beat.

| MPS | CplD size | Beats on 64-bit bus | Bus efficiency | Bus ceiling |
|---|---|---|---|---|
| 128 | 3+32 = 35 DW | 18 | 88.9% | 1.78 GB/s |
| 256 | 3+64 = 67 DW | **34** | 64/68 = 94.1% (vs 64/67 header ratio) | **1.88 GB/s** |
| 512 | 3+128 = 131 DW | 66 | 97.0% | 1.94 GB/s |

The user bus is therefore *not* the bottleneck: the link with framing is about 1.82 to 1.85 GB/s at 256 B. The margin is only about 2 to 3%, though. Any bubble between TLPs (tready gaps, header-generation dead cycles, BRAM/DDR read latency without prefetch) quickly makes your completer the limit. Design the CplD generator to stream back-to-back TLPs with data pre-fetched.

DLLP and SKP insertion inside the core deasserts tready. That loss is already counted in the link-side figures.

### Outstanding-window sizing (Little's law)

Bytes in flight = 1.8 GB/s × round-trip latency, where round trip = SSD MRd issue → your CplD fully received.

| Round-trip latency | Bytes in flight | 512 B reads outstanding |
|---|---|---|
| 0.5 µs | 0.9 KB | ≈2 |
| 1 µs | 1.8 KB | ≈4 |
| 2 µs | 3.6 KB | ≈7 |
| 4 µs | 7.2 KB | ≈14 |

Your contribution (core receive pipeline about 100 to 200 ns, DDR3 read about 100 to 300 ns, transmitting 512 B at 2 GB/s = 256 ns) gives a plausible 0.5 to 1.5 µs **[estimate]**. That is well inside what a Gen3/Gen4 SSD tolerates. The SSD's window, not yours, sets bandwidth. You only need to respond fast and never back-pressure.

---

## 8. SSD-side limits and recommended command pattern

- **Gen2 x4 caps you at about 1.7 to 1.8 GB/s.** Every modern Gen3/Gen4 drive exceeds that while writing into its SLC cache.
- **Behaviour after the cache fills is what matters:**
  - **Samsung 970 EVO Plus 1 TB:** about 42 GB of cache, then about **1.7 GB/s** (Tom's Hardware). That is close to the link rate. Smaller capacities are much lower **[UNVERIFIED]**.
  - **Samsung 970 PRO (MLC):** no cache cliff, about **2.0 GB/s sustained** (Tom's Hardware). Safe.
  - **WD Black SN770 1 TB (DRAM-less, uses HMB):** about 130 GB at about 2 GB/s, then about **400 MB/s**. Not suitable for sustained 1.6 GB/s.
  - **QLC drives** (Intel 660p, Crucial P1/P3) fall to about 100 to 150 MB/s after the cache **[UNVERIFIED figure]**. Avoid them.
  - **Good candidates for ≥1.6 GB/s sustained:** MLC drives (970 PRO); large-capacity (2 TB+) high-end TLC whose native TLC write speed is above 1.6 GB/s (980 PRO / 990 PRO class, about 2 GB/s post-cache per AnandTech and Tom's fill tests **[verify per model]**); enterprise TLC (Micron 7400/7450, Samsung PM9A3, Kioxia XD-series). Enterprise drives also add power-loss protection.
  - Dynamic SLC cache shrinks as the drive fills. Deallocate or format before each recording session (Dataset Management deallocate, opcode 09h, or admin Format NVM 80h) to restore the cache and avoid garbage-collection stalls.
- **Thermal:** consumer M.2 drives throttle at roughly 70 to 80 °C composite **[UNVERIFIED]**. Use a heatsink. Monitor the SMART/Health log (Get Log Page LID 02h; composite temperature at bytes 2:1, in K) and WCTEMP/CCTEMP.
- **Command size:** 128 KiB (MDTS ≥ 5 is near universal; Design Gateway's NVMe-IP requires MDTS ≥ 5). Use 256 KiB if MDTS allows; there is no meaningful gain at Gen2. Align SLBA to the command size and to NPWA/NOWS.
- **Queue depth:**
  - QD × 128 KiB must cover throughput × command latency. At 1.8 GB/s with 200 µs latency that is about 2.7 commands; at 500 µs it is about 7; for a 2 ms GC hiccup it is about 28.
  - Use **QD 16 to 32** with a CQ/SQ of 64 entries.
  - Design Gateway reaches 3.3 GB/s at Gen3 with only a 256 KB buffer and MQES ≥ 7, so modest QD suffices in normal conditions.
  - Put a large DDR3 ring upstream (hundreds of MB) to absorb GC stalls of tens to hundreds of ms. At 1.6 GB/s, 100 ms is 160 MB.
- **Steady-state pattern:**
  - Slot k = ring + k × 128 KiB.
  - When slot k is full and the SQ is not full: write SQE{CID=k, PRP1=slot_k, PRP2=list_k, SLBA=next_lba, NLB=255}, advance the tail, ring the doorbell, and set next_lba += 256.
  - On each CQE: check the status, free slot CID, advance the CQ head, and write the CQ head doorbell.

---

## 9. Timing, timeouts and shutdown

- **CAP.TO:** in 500 ms units. It bounds RDY 0→1 after EN=1 and RDY 1→0 after EN=0. Typical real times are much shorter: tens to hundreds of ms on cold power-up **[UNVERIFIED]**.
- **Linux timeouts:** waits `(TO+1)/2` seconds; admin command timeout 60 s; I/O command timeout 30 s.
- **Identify:** typically well under 1 ms **[UNVERIFIED]**.
- **128 KiB Write at QD1:** about 73 µs of pure transfer at 1.8 GB/s plus controller overhead, so roughly 100 to 200 µs into cache **[estimate]**. Latency grows with QD and can spike to ms or tens of ms during GC.
- **When RDY drops:**
  - after CC.EN=0;
  - CFS=1 signals a fatal error, and the host must reset;
  - PP=1 means processing is temporarily paused, for example during firmware activation.
- **Normal shutdown (§7.6.2):**
  1. Stop submitting and let outstanding commands complete.
  2. Optionally delete the I/O SQ then the I/O CQ (admin opcodes 00h and 04h, CDW10[15:0]=QID). Linux does this.
  3. Write **CC.SHN=01b** (CC=0x00464001).
  4. Poll **CSTS.SHST == 10b** (`(CSTS & 0xC) == 0x8`).

  How long to wait:
  - The spec recommends waiting at least RTD3E (bytes 88–91, µs), or 1 s if RTD3E=0.
  - Linux uses max(5 s, RTD3E), clamped to 60 s.

  After SHST=10b it is safe to remove power. Do not just clear EN (the spec says this may lengthen shutdown). To restart you need EN 1→0 followed by the full initialization.
- **Abrupt shutdown:** SHN=10b, also waiting for SHST=10b.

---

## 10. Admin command dword layouts

### Opcodes (Fig. 139)

| Opcode | Command |
|---|---|
| 00h | Delete I/O SQ |
| 01h | Create I/O SQ |
| 02h | Get Log Page |
| 04h | Delete I/O CQ |
| 05h | Create I/O CQ |
| 06h | Identify |
| 08h | Abort |
| 09h | Set Features |
| 0Ah | Get Features |
| 0Ch | AER |
| 80h | Format NVM |

### Command fields

| Command | Field | Layout |
|---|---|---|
| Identify (§5.15) | CDW10 | CNTID[31:16], rsvd[15:8], **CNS[7:0]** |
| | CNS values | 00h = namespace (NSID), 01h = controller, 02h = active NSID list |
| | CDW11 | NVMSETID[15:0] |
| | Data | 4096 B; PRP1 page aligned, no PRP list |
| Create I/O CQ (§5.3) | PRP1 | CQ base (PC=1), page aligned |
| | CDW10 | **QSIZE[31:16] (0's based)**, QID[15:0] |
| | CDW11 | IV[31:16], rsvd[15:2], **IEN[1]**, **PC[0]** |
| Create I/O SQ (§5.4) | PRP1 | SQ base |
| | CDW10 | QSIZE[31:16], QID[15:0] |
| | CDW11 | **CQID[31:16]**, rsvd[15:3], QPRIO[2:1], **PC[0]** |
| | CDW12 | NVMSETID[15:0] = 0 |
| Set Features (09h) | CDW10 | SV[31], FID[7:0] |
| Get Features (0Ah) | CDW10 | SEL[10:8], FID[7:0] |
| Number of Queues (FID 07h) | CDW11 | **NCQR[31:16], NSQR[15:0]**, both 0's based, minimum 1 queue each |
| | Completion DW0 | **NCQA[31:16], NSQA[15:0]**, 0's based |
| Volatile Write Cache (FID 06h) | CDW11 | WCE[0] |
| Get Log Page (§5.14) | CDW10 | NUMDL[31:16] (0's based dwords), RAE[15], LSP[11:8], LID[7:0] |
| | CDW11 | NUMDU[15:0] |

Worked values:
- Create I/O CQ, 64 entries, QID 1: CDW10 = **0x003F0001**, CDW11 = **0x00000001**.
- Create I/O SQ, 64 entries, QID 1 on CQ 1: CDW10 = **0x003F0001**, CDW11 = **0x00010001**.
- SMART log: NSID=FFFFFFFFh, CDW10 = 0x007F0002 (512 B).

### Other rules

- NSID **FFFFFFFFh** is the broadcast value: all namespaces, or the subsystem scope, depending on the command (§4.2 Fig. 105, §6.1). Use it only for commands that allow it (Flush if VWC bits 2:1=11b, Get Log Page for SMART, and some Set Features). Otherwise the controller returns Invalid Field.
- Commands that do not use NSID must have it 0.
- Every PCIe controller with I/O support must support at least one I/O SQ and one I/O CQ: Number of Queues requires a minimum of 1 allocated.
- Create the CQ **before** the SQ; delete the SQ before the CQ.

---

## 11. Pitfalls checklist

1. **Zero the CQs before CC.EN=1.** Expect P=1 on the first pass and flip it on every wrap.
2. **PRP2 for exactly two pages is the page address, not a list.** For more than two pages it is a list pointer. Entries other than PRP1 must have offset 0. Lists must not cross a page except through the last-entry chain.
3. **CIDs must be unique** among outstanding commands on that SQ (status 03h CID conflict otherwise). Some controllers need small CID values (Linux `NVME_QUIRK_SKIP_CID_GEN`), so use 0..QD−1.
4. **Doorbell values must stay in [0, qsize−1].** Do not write a CQ head past the posted entries. Watch for wraparound.
5. **CC.MPS must match your PRP page size** (4 KiB), and CAP.MPSMIN must be 0.
6. **The Identify buffer is 4 KiB** and page aligned. It arrives as MWr TLPs of ≤ MPS each.
7. **Write 0 to reserved CC bits.** Do not change MPS, CSS or AMS while EN=1. Do not change IOSQES/IOCQES once I/O queues exist.
8. **Configuration Request Retry Status (CRS).** After reset the SSD may answer config requests with CRS (status 010b) for up to 1 s. Wait ≥ 100 ms after PERST# deasserts before the first config request (PCIe Base §6.6.1), and retry on CRS.
9. **Command register (0x04):** set **bit 1 MSE** for BAR access and **bit 2 BME** so the SSD can issue MRd/MWr. Without BME the SSD cannot fetch the SQ, and admin commands silently never complete. Set bit 10 to disable INTx: write **0x0406**, or 0x0006.
10. **BAR0** is 64-bit memory (type 10b), non-prefetchable, at least 16 KiB (MLBAR bits 31:14 RW; §2.1.10). Some drives are larger, so size it with the all-ones write. Place it below 4 GB (BAR1 at 0x14 = 0). The 1.4 spec notes that bridges limit non-prefetchable windows to 32-bit addresses.
11. **Device Control** (PCIe cap + 0x08): MPS in bits 7:5, MRRS in 14:12, ETE in bit 8, RO in bit 4, No-Snoop in bit 11. MPS must match on both link partners. Find the capability by walking the list from config 0x34 (cap ID 10h); do not hard-code offsets.
12. **Completion timeout and UR:** answer every MRd, including unexpected addresses (with UR). Never stall completions.
13. **Keep all host structures below 4 GB.** The SSD then uses 3DW MRd/MWr headers, which keeps your parser simpler.
14. **RP bridge windows** **[UNVERIFIED, check PG054]:** set the 7-series RP's own Command register BME/MSE and bus numbers (0/1/1). Keep the host-buffer addresses *outside* the RP's downstream memory base/limit window, so upstream MRds are forwarded to user logic and not treated as peer traffic.
15. **Endianness** **[UNVERIFIED]:** NVMe structures are little-endian. Xilinx 7-series AXI-S TLP header dwords versus payload byte lanes have documented orderings; verify against PG054 and simulation.
16. **Namespace Not Ready** (SC 82h, DNR=0) can occur right after RDY=1. Retry after a delay.
17. **Link speed:** if Link Status (+0x12) bits 3:0 read 1 (2.5 GT/s), set the RP's Link Control 2 target speed to 2 and retrain (Link Control bit 5) **[pcie_7x RP specifics UNVERIFIED]**. Leave ASPM off (Link Control bits 1:0 = 00).
18. **DRAM-less drives** ask for an HMB (HMPRE/HMMIN). Providing none is legal but lowers performance.
19. **Writes beyond NSZE** fail with LBA Out of Range (SC 80h). NLB is 0's based.
20. **64-bit registers:** do aligned 32-bit accesses, low dword first (as Linux does). Never an access that spans two registers.

---

## 12. Boot to first write in 25 steps

Example addresses: BAR0=0xC000_0000, ASQ=0x9010_0000, ACQ=0x9010_1000, Identify buffer=0x9010_2000, IOSQ=0x9010_3000, IOCQ=0x9010_4000, PRP lists=0x9000_0000, data ring=0x8000_0000. SSD at bus 1, device 0, function 0; RP at 00:00.0.

1. Power the M.2 3.3 V rail (up to about 2.5 to 3 A). Run a 100 MHz REFCLK. Hold PERST# low for ≥ 100 ms after power is stable, with REFCLK stable ≥ 100 µs before release.
2. Release PERST#. Wait for `user_lnk_up`/L0. Check the RP Link Status for 5 GT/s x4; force a speed change if needed.
3. Wait ≥ 100 ms after PERST# before the first config request. Retry on CRS for up to 1 s.
4. RP's own config space (cfg_mgmt): Command = 0x0006, bus numbers primary=0, secondary=1, subordinate=1, RP Device Control MPS = the chosen value.
5. CfgRd0 at 0x00: Vendor/Device ID must not be 0xFFFF. CfgRd0 at 0x08: class code 0x010802 (mass storage, NVM, NVMe).
6. BAR sizing: CfgWr 0x10 = 0xFFFFFFFF and read back (for example 0xFFFFC004 = 16 KiB, 64-bit, non-prefetchable). CfgWr 0x14 = 0xFFFFFFFF and read back.
7. CfgWr 0x10 = 0xC000_0000 and 0x14 = 0x0000_0000.
8. Walk the capabilities from 0x34 to ID 10h (PCIe). Read Device Capabilities (+0x04): MPSS in bits 2:0, ETFS in bit 5.
9. CfgWr Device Control (+0x08), for example 0x5130 for MRRS=101b (4 KiB), ETE=1, MPS=001b (256 B), RO=1. Use 0x2130 for MRRS=512. Leave the error-reporting enables at 0 if you do not handle error messages.
10. CfgWr Link Control (+0x10): ASPM=00. Optionally set Common Clock Configuration and retrain. Read Link Status to confirm Gen2 x4.
11. CfgWr Command (0x04) = **0x0406** (MSE + BME + INTx disable).
12. MRd BAR0+0x00 and +0x04 (CAP). Check MPSMIN=0 and CSS bit 37. Latch MQES, TO, DSTRD. MRd +0x08 (VS).
13. MRd CC. If EN=1: MWr CC=0, then poll CSTS.RDY=0 (timeout TO×500 ms).
14. Zero the ACQ (and the IOCQ) RAM. Zeroing the SQs is optional.
15. MWr AQA=0x001F001F, ASQ=0x9010_0000 (lo then hi), ACQ=0x9010_1000 (lo then hi).
16. MWr CC=0x00460000, then CC=0x00460001.
17. Poll CSTS until RDY=1 and CFS=0 (timeout TO×500 ms).
18. Build an Identify Controller SQE at ASQ[0]: CDW0=0x00000006 (CID 0), NSID=0, PRP1=0x9010_2000, PRP2=0, CDW10=0x00000001. MWr SQ0TDBL (0x1000) = 1.
19. Wait for ACQ[0].P==1 with status 0. MWr CQ0HDBL (0x1000 + (4<<DSTRD)) = 1. Parse MDTS(77), SQES(512)=0x66, CQES(513)=0x44, NN(516), VWC(525), RTD3E(88).
20. Identify Namespace: CID 1, NSID=1, CDW10=0. Ring the doorbell with tail=2. Read NSZE(0), NLBAF(25), FLBAS(26), LBAF at 128+4·(FLBAS&0xF): LBADS = bits 23:16, MS must be 0. Write the CQ head.
21. Set Features Number of Queues: opcode 09h, CDW10=0x07, CDW11=0. Check completion DW0 ≥ 0 for both queue counts.
22. Create I/O CQ: opcode 05h, PRP1=0x9010_4000, CDW10=0x003F0001, CDW11=0x00000001.
23. Create I/O SQ: opcode 01h, PRP1=0x9010_3000, CDW10=0x003F0001, CDW11=0x00010001.
24. Optional: Get Features VWC (0Ah, FID 06h) and the SMART log (LID 02h) for temperature.
25. Write streaming:
    - Fill IOSQ[t] with the Write SQE from §3, advance t, MWr SQ1TDBL (0x1008 when DSTRD=0) = t.
    - Serve the SSD's MRds for the SQE, PRP list and data.
    - Consume IOCQ CQEs with phase tracking and MWr CQ1HDBL (0x100C).
    - Repeat, keeping QD commands in flight.

    At the end:
    - Flush (NVM opcode 00h, NSID=1).
    - Delete SQ 1, then CQ 1.
    - CC=0x00464001.
    - Poll SHST=10b for up to max(RTD3E, 5 s).
    - Power off.

---

## 13. Items to verify on your hardware

- How many MRds the actual SSD keeps outstanding, and their sizes at your chosen MRRS (ILA on the pcie_7x receive interface).
- MPSS and BAR0 size for your specific drive.
- pcie_7x Root Port details: NP credit configuration, bridge-window forwarding behaviour, speed-change procedure, and AXI-S byte ordering (PG054).
- Post-cache sustained write speed for your exact SSD model and capacity, measured by a long fill test with the drive at temperature.
- Spec citations are NVMe 1.4. In NVMe 2.0 the content was split: Base 2.0 holds initialization (around §3.5) and queues, the NVM Command Set spec 1.0 holds Write and Flush, and the NVMe over PCIe Transport spec 1.0 holds the PCI header, doorbells and ETFS. The 2.0 section numbers are approximate and **[UNVERIFIED]**.

## Sources

- NVM Express Base Specification 1.4: https://nvmexpress.org/wp-content/uploads/NVM-Express-1_4-2019.06.10-Ratified.pdf
- NVMe over PCIe Transport Specification 1.0a: https://nvmexpress.org/wp-content/uploads/NVMe-PCIe-Transport-Specification-1.0a-2021.07.26-Ratified.pdf
- Linux NVMe driver: https://github.com/torvalds/linux/blob/master/drivers/nvme/host/core.c , pci.c , nvme.h ; https://github.com/torvalds/linux/blob/master/include/linux/nvme.h
- Design Gateway NVMeG3-IP datasheet: https://dgway.com/products/IP/NVMe-IP/dg_nvmeg3_ip_data_sheet_xilinx_en/dg_nvmeg3_ip_data_sheet_xilinx_en.htm
- MLE, AXI Bridge NP-credit bottleneck: https://www.missinglinkelectronics.com/devzone/fpga-devtips/a-deep-dive-into-xilinx-axi-bridge-for-pci-express-xilinx-pg194-and-why-we-tweaked-c_m_axi_num_readq/
- MLE, recording with NVMe SSDs: https://www.missinglinkelectronics.com/devzone/application-notes/recording-with-nvme-ssds/
- Tom's Hardware, 970 EVO Plus: https://www.tomshardware.com/reviews/samsung-970-evo-plus-ssd,5608-4.html
- Tom's Hardware, 970 PRO: https://www.tomshardware.com/reviews/samsung-970-pro-ssd-review,5572-2.html
- SN770 1 TB: https://www.pcgamer.com/au/wd-black-sn770-performance-review , https://www.tomshardware.com/reviews/wd-black-sn770-ssd-review/3
- PCIe split completions and tags: https://www.vlsimentor.com/pcie/host-memory-access , AMD AR 36591 https://adaptivesupport.amd.com/s/article/36591
