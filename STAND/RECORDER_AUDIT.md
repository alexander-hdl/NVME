# RECORDER_AUDIT — interface audit before WP8 integration

Date 2026-11-24. Scope: the ports in the files as they are now (not spec 4.4 text) of WP0–WP7 + WP9
tools, against `STAND/RECORDER_SPEC.md` v0.2, `CODING_RULES.md`, `INTEGRATION_NOTES.md` (WP0–WP9).
MIG user ports are taken from the generated IP `fpga/build/mig_check/ip/mig_ddr3/mig_ddr3/user_design/rtl/mig_ddr3.v`.
Nothing was simulated or built; `rec_decode.py decode` was run on the four block vector files (all decode).

Marks in the tables: **W** width mismatch, **D** domain mismatch, **ND** no driver (glue needed in the top),
**U** unused output, **S** semantic mismatch, **OK** matches. `Dn` = defect number in section 2.

---

## 1. Connection table for `rtl/board/ax7a200b_rec_top.sv`

### 1.1 Clocks and resets (reset contract spec 4.2/4.3, CODING_RULES 4)

| Net | Source | Destination(s) | Domain | Note |
|---|---|---|---|---|
| `link_clk` | `nvme_host_top.link_clk_out` (free_clk R4/T4 → IBUFDS → BUFG) | all link modules, `rec_cdc_bridge.link_clk`, `tdp_ram_2clk.a_clk`, `mig_ddr3.sys_clk_i` | link 200 MHz | OK. MIG "No Buffer" from BUFG; `free_clk` create_clock covers the MIG PLL (generated) |
| `ui_clk` | `mig_ddr3.ui_clk` (or `mig_app_model.ui_clk` when `USE_MIG_MODEL=1`) | rec_profile, rec_framer, ddr_fifo, rec_sink_chk, rec_verify, `vrf_tap.vclk`, bridge `ui_clk`, `tdp_ram_2clk.b_clk`, `nvme_host_top.data_clk` | ui 100 MHz | OK |
| `user_clk` | `nvme_host_top.user_clk_out` | `vrf_tap.uclk`, bridge `user_clk` | user 125 MHz | OK |
| `rec_rst` | POR counter (link) OR sync(`rst_key_n`) | uart_rx, rec_cmd_parser, rec_tx_fmt, rec_ctrl, xadc_mon, bridge `link_rst` | link, sync, act. 1 | ND (WP8 glue: POR counter + 2FF sync of the key) |
| `core_rst_n` | `~rec_rst_key & ~vio.probe_out0 & ~rec_ctrl.core_rst_req` | `nvme_host_top.board_rst_n` (async input) | link | ND. `core_rst_req` is a registered 1 ms level (`MS_CYC`) — OK. Resets link_bringup → clears `link_lost/link_fail`, PERST cycle → `user_reset` |
| `mig_sys_rst_n` | POR only (link) | `mig_ddr3.sys_rst` (active low, per .prj) | link | ND. Not present on `mig_app_model` (model has no sys_rst) |
| `ui_rst` | `mig.ui_clk_sync_rst` OR 2FF-sync(`rec_rst`) into ui_clk | all ui modules, bridge `ui_rst`, `vrf_tap.vrst` | ui | ND (top reset synchroniser, async assert allowed only here) |
| `user_rst` | **does not exist on `nvme_host_top`** (internal `user_reset`, not exported) | `vrf_tap.urst`, bridge `user_rst` | user | **ND → D2 (BLOCKER)** |
| `ui_ok` | `~ui_rst` registered | `ui_stat_in.ui_ok` | ui | ND. Note: ui_stat does not update while `ui_rst=1` (cdc_bus src in reset), so a fresh `ui_stat.ui_ok` is always 1; the real indicator is `ui_age_ok` (OK, harmless) |

### 1.2 link_clk domain

| From (module.port) | To (module.port) | Width | Via | Mark |
|---|---|---|---|---|
| pin `uart_rx` (L14) | `uart_rx.rxd` | 1 | 2FF inside uart_rx (X12) | OK |
| `uart_rx.m_data / m_valid / frame_err` | `rec_cmd_parser.rx_data / rx_valid / rx_ferr` | 8/1/1 | — | OK |
| `rec_cmd_parser.cmd_valid, cmd_op[5:0], cmd_sub[3:0], cmd_nargs[1:0], cmd_arg0..2[63:0], cmd_err[7:0]` | `rec_ctrl.cmd_*` same names | 1/6/4/2/64×3/8 | — | OK, encodings identical (item n) |
| `rec_ctrl.cmd_ready` | `rec_cmd_parser.cmd_ready` | 1 | — | OK (low while the reply is built: ERR 0C rule works) |
| `rec_ctrl.msg_valid, msg_kind[4:0], msg_code[7:0], msg_arg0/1[63:0], snap (rec_snap_t 2884), tel_tick` | `rec_tx_fmt.*` same names | | — | OK (see D10 for the snap copy) |
| `rec_tx_fmt.msg_ready` | `rec_ctrl.msg_ready` | 1 | — | OK |
| `rec_tx_fmt.txd` (uart_tx inside) | pin `uart_tx` (L15) | 1 | — | OK; `uart_tx` is NOT instantiated in the top. Set `BLD_DATE` per build |
| KEY1..4 `key_n[3:0]` | debounce → 1-clk pulses → `rec_ctrl.key_rec, key_stop, key_pf, key_page` | 1 each | 2FF + debounce (WP8) | ND |
| VIO `probe_out1/2` (levels, link_clk) | rising-edge → `rec_ctrl.vio_start, vio_stop` | 1 | edge detect (WP8) | ND / S (rec_ctrl expects pulses) |
| VIO `probe_out6` (level) | `rec_ctrl.vio_verify` | 1 | — | OK (level is what rec_ctrl wants) |
| `rec_ctrl.led[3:0]` (1 = on) | `led_n[3:0] = ~led` | 4 | — | OK |
| `xadc_mon.xadc` (`xadc_t`, 49) | `rec_ctrl.xadc` | 49 | — | OK |
| `xadc_mon.xadc.temp` | `rec_cdc_bridge.xadc_temp` | 12 | X14 cdc_bus repeat → ui | OK |
| `xadc_mon.upd` | — | 1 | | U (fine) |
| `rec_ctrl.prof_we, prof_addr[5:0], prof_wdata[71:0]` | `tdp_ram_2clk(W=72,DEPTH=64).a_we, a_addr, a_wdata` | 1/6/72 | X3 | OK |
| `tdp_ram_2clk.a_rdata` | `rec_ctrl.prof_rdata` | 72 | latency 2 (rec_ctrl waits 4) | OK |
| `rec_ctrl.cfg, step, step_load, start_cfg[40:0], start_load, src_stop, src_pf, ui_clear, mt_start, dtest_start, log_req` | `rec_cdc_bridge.*` same names | 115/32/1/41/1/1×6 | X1/X2/X2a/X2b/X8 | OK |
| `rec_cdc_bridge.ui_stat, ui_age_ok, core_stat, core_age_ok` | `rec_ctrl.*` | 1245/1/823/1 | X4/X10 | OK |
| `rec_ctrl.ctrl_start, ctrl_stop, verify_mode, format_req, verify_cmds[31:0]` | `nvme_host_top.*` same names | 1/1/1/1/32 | X7: 2FF per bit inside `nvme_host_top` | OK (top syncs; `tap_verify_mode` is the synced copy) |
| `nvme_host_top.link_lost, link_fail` (link, sticky) | **no input on rec_ctrl** | 1/1 | — | U / **S → D3** (item a) |
| `rec_ctrl.t_ms` | — | 32 | | U (value goes through `snap`) |
| core status for LED/VIO (X11) | VIO probes as in `ax7a200b_top`; `probe_in19/20/21` = `ui_stat.fill_max_bytes>>4`, `fill_bytes>>4`, `drop_blocks` from the bridge output (link) | | | ND (WP8) |

### 1.3 ui_clk domain

| From | To | Width | Via | Mark |
|---|---|---|---|---|
| bridge `cfg_ui.bp / sid / rsid / ddr_mib / prof_n / prof_rep / prof_end / mt_mib / lbads` | `rec_framer.bp_mode`, `rec_framer.sid` + `rec_sink_chk.exp_sid`, `rec_verify.exp_sid`, `ddr_fifo.size_mib`, `rec_profile.nseg / rep / end_stop`, `ddr_fifo.mt_mib`, `rec_verify.lbads` | 1/32/32/11/7/16/1/11/4 | X2 repeat | OK. `cfg.sid` = predicted SID in IDLE, `sid_use` in a session; framer samples at start → same value the sink checks (OK) |
| bridge `step_ui`, `rate_upd` | `rec_profile.const_step, rate_upd` | 32/1 | X2a | OK |
| bridge `start_cfg_ui[40:1]`, `start_cfg_ui[0]` | `rec_framer.len_blocks`, `rec_framer.prof_en` + `rec_profile.prof_en` | 40/1 | X2b (held data, valid with `src_start`) | **S → D1**: stale for DTEST |
| bridge `src_start` | `rec_framer.start`, `rec_profile.start` | 1 | X2b dst_upd | **ND for DTEST → D1** |
| bridge `src_stop_ui`, `src_pf_ui` | `rec_framer.stop / pf`; `rec_profile.stop = src_stop_ui \| src_pf_ui` | 1 | X1 | OK |
| bridge `ui_clear_ui` | **one net** to `rec_profile.clear, rec_framer.clear, ddr_fifo.clear, rec_verify.clear, rec_sink_chk.clear`, DTEST latch | 1 | X1 | OK if fanned out from the single pulse (item h) |
| bridge `mt_start_ui` | `ddr_fifo.mt_start` | 1 | X1 | OK |
| bridge `dtest_start_ui` | (nothing in WP3/WP4 takes it) | 1 | X1 | **ND → D1** |
| bridge `device_temp` | `mig_ddr3.device_temp_i` (MIG tempmon samples it in its fabric clk = ui) | 12 | X14 | OK |
| `rec_profile.rd_en, rd_addr[5:0]` | `tdp_ram_2clk.b_en, b_addr` | 1/6 | X3 | OK |
| `tdp_ram_2clk.b_rdata` | `rec_profile.rd_data` | 72 | latency 2 | OK |
| `rec_profile.step, seg_idx[5:0], rep_cnt[15:0]` | `rec_framer.step, seg_idx, rep_cnt` and `ui_stat_in.step, seg_idx, rep_cnt` | 32/6/16 | — | OK |
| `rec_profile.prof_active, prof_end_s` | `ui_stat_in.prof_active, prof_end_s` | 1/1 | — | OK |
| `rec_profile.prof_end` (pulse) | — | 1 | | U (level `prof_end_s` used) |
| `rec_framer.m_tdata, m_tvalid` / `ddr_fifo.s_tready` | `ddr_fifo.s_tdata, s_tvalid` / `rec_framer.m_tready` | 128/1/1 | — | OK |
| `ddr_fifo.space_ok` | `rec_framer.space_ok` | 1 | — | OK |
| `ddr_fifo.fill_bytes[30:10]` | `rec_framer.ddr_fill_kib = {11'b0, fill_bytes[30:10]}` | 31→32 | — | **W** (glue: shift + zero extend) |
| `rec_framer.src_blocks, wr_blocks, drop_blocks, pad_blocks, trunc_blocks, bp_max_beats, rt_stall_max, len_reached_s, done` | `ui_stat_in.*` (`done` → `src_done`) | 48/48/32/16/16/40/16/1/1 | X4 | OK (`rt_stall_max` reaches link but nobody prints it: U) |
| `rec_framer.active` | `ui_stat_in.src_active` | 1 | X4 | **S → D4** (item d) |
| `rec_framer.len_reached` (pulse) | — | 1 | | U |
| `ddr_fifo.m_tdata, m_tvalid` | REC: `nvme_host_top.s_data_tdata/tvalid`; DTEST: `rec_sink_chk.s_tdata/s_tvalid` | 128/1 | X6 = `data_ring` async FIFO (`data_clk = ui_clk`) | **ND → D1** (mux + `s_data_tvalid` gate by DTEST latch) |
| `nvme_host_top.s_data_tready` / `rec_sink_chk.s_tready` (=1) | `ddr_fifo.m_tready` (muxed) | 1 | | ND (same mux) |
| `ddr_fifo.fill_bytes, fill_max_bytes, idle, wr_bytes, rd_bytes` | `ui_stat_in.fill_bytes, fill_max_bytes, ddr_idle, ddr_wr_bytes, ddr_rd_bytes` | 31/31/1/48/48 | X4 | OK (`ddr_wr/rd_bytes` U at rec_ctrl) |
| `ddr_fifo.mt_busy, mt_done, mt_err_cnt, mt_first_err_addr` | `ui_stat_in.mt_*` | 1/1/32/29 | X4 | OK (`mt_done` = sticky & masked from the start pulse: item e satisfied) |
| `rec_sink_chk.beats, hdr_err` | `ui_stat_in.dt_beats, dt_hdr_err` | 48/32 | X4 | OK (`dt_beats` U) |
| `vrf_tap.m_tdata, m_tvalid` / `rec_verify.s_tready` | `rec_verify.s_tdata, s_tvalid` / `vrf_tap.m_tready` | 193/1/1 | X15 inside vrf_tap | OK |
| `vrf_tap.ovf` (ui, via cdc_sync) | `rec_verify.tap_ovf` | 1 | — | OK |
| `rec_verify.res, end_found` | `ui_stat_in.vrf, end_found` | 665/1 | X4 | OK |
| `rec_verify.mvp_flags` | — | 8 | | U (no LAST indication in `!RV`, see D17) |
| `mig.init_calib_complete` | `ddr_fifo.init_calib_complete`, `ui_stat_in.calib` | 1 | X4 | OK |

### 1.4 MIG native port (`mig_ddr3`, ui_clk) ↔ `ddr_fifo`

| ddr_fifo | mig_ddr3 | Width | Mark |
|---|---|---|---|
| `app_addr, app_cmd, app_en` → | `app_addr, app_cmd, app_en` | 29/3/1 | OK. `app_en`/`app_wdf_wren`/`app_wdf_end` are combinational from `app_rdy`/`app_wdf_rdy` (D14) |
| `app_wdf_data, app_wdf_mask, app_wdf_wren, app_wdf_end` → | same | 256/32/1/1 | OK (mask 0) |
| ← `app_rdy, app_wdf_rdy, app_rd_data, app_rd_data_valid, app_rd_data_end` | same | 1/1/256/1/1 | OK (`app_rd_data_end` unused inside) |
| `app_sr_req, app_ref_req, app_zq_req` (= 0) → | same | 1×3 | OK |
| — | `app_sr_active, app_ref_ack, app_zq_ack, device_temp[11:0], calib_tap_req` (outputs) | | U |
| — | `calib_tap_load, calib_tap_addr[6:0], calib_tap_val[7:0], calib_tap_load_done` (inputs) | | **ND**: tie to 0 (not in WP4 notes; not on `mig_app_model`) |
| pins `ddr3_*` | `mig_ddr3.ddr3_*` | as spec 4.4 WP8 | OK (71 pins checked by mig_ip.tcl) |

`USE_MIG_MODEL=1`: `mig_app_model` has the same app/ui ports but no `sys_clk_i`, `sys_rst`, `calib_tap_*`, `device_temp`, DDR pins → wrap both in a generate in the top.

### 1.5 user_clk domain / core (`nvme_host_top`)

| From | To | Width | Via | Mark |
|---|---|---|---|---|
| bridge `log_req_u` (user pulse) | `nvme_host_top.log_req` | 1 | X8 | OK (core gates it to RUN/DRAIN) |
| `nvme_host_top.tap_wr_en, tap_wr_addr[31:0], tap_wr_data[127:0], tap_wr_be[15:0], tap_slot_lba_flat[N_SLOTS*64], tap_verify_mode` | `vrf_tap.wr_en, wr_beat_addr, wr_data, wr_be, slot_lba_flat, enable` | 1/32/128/16/N×64/1 | — | OK (item p): names differ only by spec text; `tap_wr_addr` = DATA-relative byte address = what vrf_tap slices (`[OFFW +: SW]`). `vrf_tap #(N_SLOTS, SLOT_BYTES)` and `rec_verify #(START_LBA)` must take the same values as `nvme_host_top` (core default `N_SLOTS_DEF=8`, spec top default 4); `rec_verify.N_CTX (8) >= N_SLOTS` required |
| core status → `core_stat_in` (assembled in user domain): `seq_state=stat_seq_state, error=status_error, error_detail=status_error_detail, flags=status_flags, link_up=status_link_up, gen2=status_gen2, link_width=status_link_width, lba_range_err=status_lba_range_err, bytes_done=stat_bytes_done, cmds_done=stat_cmds_done, cmd_errors=stat_cmd_errors, max_latency=stat_max_latency, beats_in=stat_beats_in, lbads=stat_lbads, nsze=stat_nsze, smart=smart_t'(log_smart), log_cnt, log_err_cnt, log_busy, log_status, rtd3e_us, cmd_limit_done` | bridge `core_stat_in` | 823 | X10 | OK, all user_clk registers (beats_in counted in data_ring read side = user) |
| `nvme_host_top.format_done, format_ms, log_done, log_err, vfy_*` (VERIFY_INTERNAL=0 → 0), `stat_cpl_evt/lat, stat_rx_msgs, stat_first_error_lba, link_retries, dbg_ltssm_state, dbg_uc` | VIO/ILA or nothing | | | U (fine; `format_ms` could replace rec_ctrl's own `rf_ms`, not needed) |
| `rec_ctrl.snap.rtd` ← `core_stat.rtd3e_us` | | 32 → 16 | | **W → D6** (item b) |
| `core_stat.link_width` → `$T lk[3:2]` | | 2 | | **S → D16**: it is `pl_sel_lnk_width` (log2 lanes), spec says "width-1"; recorder.py uses `+1` (right for x1/x2 only) |

---

## 2. Defect list

Owner = package that changes code. "Spec" = orchestrator/WP10 edits `RECORDER_SPEC.md` (v0.3).

### BLOCKER (must land before `tb_rec_top` runs S1–S12)

| # | Item | Defect | Owner | Fix |
|---|---|---|---|---|
| D1 | — | **DTEST cannot start.** rec_ctrl LOCAL/DTEST issues only `dtest_start` (X1); `rec_framer.start`/`rec_profile.start` are driven by `src_start` (X2b) only, and `start_cfg_ui` still holds `len_blocks/prof_en` of the previous REC. Nothing selects `rec_sink_chk` as the sink or keeps `data_ring` idle. S9 fails. | WP2 + WP8 | WP2: at `lt_cnt_q == CLR_DLY` in DTEST also `start_cfg <= 41'd0; start_load <= 1` (one-shot X2b → `src_start`, len 0, const step). WP8: ui latch `dt_mode_q` (set `dtest_start_ui`, clear `ui_clear_ui`/`ui_rst`) muxes `ddr_fifo.m_*` to `rec_sink_chk` and forces `nvme_host_top.s_data_tvalid=0`. |
| D2 | — | **No user-domain reset on `nvme_host_top`.** `vrf_tap.urst` (FIFO write side, ovf) and `rec_cdc_bridge.user_rst` (X8 dst, X10 src) need the IP's `user_reset`, which is internal. | WP5 | Add `output logic user_rst_out` (= `user_reset`, user_clk) to `nvme_host_top` (and pass-through in kc705/ax7a200b tops unconnected). |

### IMPORTANT (fix before the scenario that exercises it; listed in 3)

| # | Item | Defect | Owner | Fix |
|---|---|---|---|---|
| D3 | a | **rec_ctrl hangs on link loss / dead user_clk.** link_lost: sequencer → ERROR → IP `user_reset` → cs=IDLE (E_LINK_LOST); if X10 misses the short ERROR, rec_ctrl sits in RUN/SRC_STOP/CORE_STOP (no rule for cs=IDLE). During PERST/link_fail `user_clk` is dead → `core_age_ok=0` forever in BRINGUP/RUN/…/RELEASE. Only ABORT recovers. | WP2 (+WP8 wire, Spec 1.3/4.4) | New rec_ctrl inputs `link_lost, link_fail` (link domain, sticky, direct from `nvme_host_top`, no CDC). In BRINGUP…RELEASE: `link_lost \| link_fail` → ERROR (→ CORE_RST, which clears both via `core_rst_n`). Plus: in RUN/SRC_STOP/CORE_STOP a fresh snapshot with `cs==CS_IDLE` → ERROR; watchdog `core_age_ok==0` for > 3 s (> link timeout 2 s + PERST 0.2 s) in BRINGUP…RELEASE → ERROR. `!E st=` = state at the event. |
| D4 | d | **`src_active` does not fall at stop/pf.** `rec_framer.active = run_q && !done_q` → stays 1 until `done`; WP2 then gets t1 = t2 (phase 1 of `!RP` lost). | WP3 | `assign active = run_q && !fin_q && !done_q;` (NCO running; `fin_q` is set by stop/pf/len). Header comment + note; no other user of `active`. |
| D5 | — | **MEMTEST STOP / ABORT leave the memtest running.** WP2 aborts MEMTEST by `ui_clear`; WP4: `clear` does not touch a running memtest. The test keeps owning the MIG and destroys DDR; next REC sees `space_ok=0` (RT drops everything) until it ends (~2 s/GiB). | WP4 | `ddr_memtest`: `clear` aborts (stop issuing, drain own reads in flight, `busy=0`, `done` stays 0, counters kept); `ddr_fifo` passes `clear` to it. Test: abort mid-pass, then a REC stream. (Alternative WP2: STOP in MEMTEST waits for `!mt_busy` — rejected, keys/ABORT would still race.) |
| D6 | b | `rec_snap_t.rtd` 16 bit, RTD3E in µs (typ. 0x186A0…0xF4240) → saturated `FFFF` in `!RP rtd` and `SMART? rtd`. | WP0 → WP2 | WP0: `logic [31:0] rtd;` (snap 2900 bit). WP2: `snap_live.rtd = core_stat.rtd3e_us` (drop saturation). WP1: nothing (field h8 of `snap_q.rtd`). Rerun tb_rec_tx_fmt/tb_rec_ctrl. |
| D7 | — | **SRC_STOP hangs when STOP/PF/key arrives before `start_load`** (RUN entry + ≤ ~25 clk of the capacity engine, `rs_q` 0..3 → 5): src never started, `ui_stat.src_done` never rises, drain rule never true. | WP2 | In SRC_STOP/CORE_STOP treat `rs_q == 5` as "source done" (`src_done_eff = ui_stat.src_done \|\| rs_q==5`; t1/t2 = t0). Add to tb_rec_ctrl. |
| D8 | m | WP6 never ran `rec_verify` on WP3 output. Field layout checked here line by line (beat0 `{16'0,pos48,sid,magic}`, beat1 `{ts,16'0,seq48}`, beat2 `{kib,lost,valid,rep,10'0,seg6,flags,ver}`, beat255 `{magic_end,~crc,pos_lo,seq_lo}`, pattern on TRUNC/PAD tails, empty TRUNC `valid=0`, `seq−lost=pos` incl. PAD/TRUNC, `$readmemh` 128-bit MSB-first in both) → compatible; contexts keyed by `pos[5+:3]` fit 128 KiB commands from START_LBA with N_SLOTS ≤ 8. Risk is only "untested". | WP6 | Extend `tb_rec_verify`: feed `blocks_stop/pf_drop/len.hex` (sid 692D3A00, lbads 9 and 12) → expect `ok=32, bad=0, pad=25/…, mvp=31, fbp=all-ones`. |

### MINOR (can wait until after first S-runs)

| # | Item | Defect / decision | Owner | Fix |
|---|---|---|---|---|
| D9 | c | 2884-bit `snap` copy in rec_tx_fmt (3.7 k FF) + `snap` register (2.9 k) + `res_q` (2.9 k) in rec_ctrl. Dropping the formatter copy needs a new handshake (formatter also latches `snap` for `tel_tick` at a moment rec_ctrl cannot see). **Decision: keep the copy for v1** (≈ 9 k FF of 269 k; spec 8.1 FF total stays ~11 %). Optional later: make rec_ctrl `snap` a combinational mux of registers (−2.9 k FF, waiver of "outputs registered"). | — / Spec 8.1 | Update budget line "UART+parser+fmt+ctrl" to ≈ 10 k FF. |
| D10 | g | `vrf_tap.ovf` clears only while `enable=0`. rec_ctrl drops `verify_mode` in RELEASE/ERROR/ABORT → OK, but a VIO READ LAST right after RELEASE→IDLE keeps `verify_mode=0` for ~1 link clk, not enough to reach user_clk; the old ovf re-sets `res.ovf`. | WP2 | Block `go_read` for 256 link clk after `verify_mode` falls (counter), or emit `ui_clear` for READ ≥ 1 µs after `verify_mode` rises. |
| D11 | f | Stale `nsze` across CORE_RST: `nvme_admin.nsze/lbads` reset only by `user_rst`; CORE_RST → PERST → `user_reset` clears them, so a post-CORE_RST BRINGUP waits for a fresh Identify. Without CORE_RST the old value is the same SSD's (hot swap not supported). `verify_cmds` for READ END may load after RUN entry, but `cmd_limit_done` compares live, so a late load is harmless. **No change**; document in 1.3 ("same SSD per power-on"). | Spec | — |
| D12 | h | Satisfied if WP8 fans out ONE `ui_clear_ui` net (framer, profile, ddr_fifo, verify, sink, DTEST latch). `size_mib` changes only via DDRSZ (IDLE) and every session starts with `ui_clear`; all volumes are whole 4 KiB blocks ending on 128 KiB → multiples of 2 KiB. | WP8 | Assertion in tb_rec_top: `ddr_fifo.idle` before each `src_start`. |
| D13 | i | Counter semantics: `wr/rd/fill_bytes` = stream level (WP4) = what spec 2.5 `ddr/ddrm` and `!RP pdd` mean (FIFO incl. staging/output) → OK; `ddr_fill_kib = fill_bytes>>10` (glue). `bp_max_beats` (`$T bpm`) is now "max backlog of a completed block, 0…256", BP wait not counted; `rt_stall_max` not reported anywhere. recorder.py prints `bpm` raw → no code mismatch, only spec text. | Spec 2.5, 4.4 | Rewrite `bpm` row; drop `rt_stall_max` from 4.4 or add to `!RR`. |
| D14 | q | `app_en/app_wdf_wren/app_wdf_end` combinational on `app_rdy/app_wdf_rdy` (WP4 waiver of "outputs registered"). Few LUTs inside ui_clk 10 ns into MIG `ui_cmd` input regs. | WP8 (B0) | Check `report_timing -from app_rdy` at B0; if WNS < 0.5 ns: WP4 adds hold-until-ready registered `app_en`. Add waiver line to INTEGRATION_NOTES. |
| D15 | j | NCO step rounding. **Decision: `round()`** (formula 2.3 is normative; recorder.py already rounds; rec_ctrl/framer take the step as given; `STEP_DEF 10000000` is exact). | Spec 2.7 | Example: `PROF ADD 2CCCCCCD …`, fixed-width form. Nobody else changes. |
| D16 | — | `$T lk[3:2]` = `pl_sel_lnk_width` (00 x1, 01 x2, 10 x4), spec says "width−1"; recorder.py `lanes = code+1` wrong for x4. | WP9 + Spec 2.5 | `lanes = 1 << code`; spec "b3:2 log2(lanes)". |
| D17 | — | `rec_verify.mvp_flags` unused: `!RV` cannot say "chain ends with LAST" (spec 3.2). | Spec / WP2 later | Optional `lfl=h2` in `!RV` (needs snap/vrf_res field). Not for v1. |
| D18 | k | `crc_vectors.hex` has 9 vectors (`// vectors: 9`, generator: 3 patterns + blocks); WP9 note says 11. tb_rec_framer reads the count from the file. **Harmless.** | WP9 | Correct note 9 (N=9). |
| D19 | l | recorder.py `FakeDevice`: `READ SID` replies `OK READ SID` (rejected by `parse_line`), `MEMTEST` replies bare `OK MEMTEST`; RTL sends `OK READ rsid=h8`, `OK MEMTEST mib=h4`. Also `!RV ... fbp=FFFFFFFFFF end=1` is self-contradictory (end = fbp exists). | WP9 | Fix both replies, add `"READ": [[], _spec("rsid:8")]`, `"MEMTEST": [[], _spec("mib:4")]` to `OK_SPEC`; fake `!RV`: `fbp=wr_blocks`, `end=1`. |
| D20 | o | XADC: raw 12-bit codes, formulas identical in xadc_mon notes and recorder.py (`*503.975/4096−273.15`, `*3/4096`). OK, no action. INIT_41 alarm bits to confirm at B2. | WP7 (B2) | — |
| D21 | n | Parser ↔ rec_ctrl: opcodes (`rec_op_e`), `cmd_sub` (MODE 0 RT/1 BP, PROF END 0 HOLD/1 STOP), arg order (PROF ADD step,dur; PROF SET i,step,dur; MEMTEST `nargs==0` = default), `OP_PERR`+`cmd_err` all consistent. No action. | — | — |
| D22 | — | `PING ver` = `REC_VER` 01, spec example shows 02; `$T v=01`. | Spec 2.7 | Example → `ver=01`. |
| D23 | — | Spec 4.4 lists no `link_clk/link_rst` on the bridge, no `prof_end_s/len_reached_s/mvp_flags`, `ddr_fifo` internal layout 256×512 (WP0/WP3/WP4/WP6 deviations). | Spec 4.4 | Fold the logged deviations into v0.3. |

Items checked and found consistent (no defect): (e) `mt_done` sticky until the next start and masked from the start pulse — stays valid until the next `ui_clear`/start, rec_ctrl waits 256 clk after `mt_start`; (p) tap ports; X2b/X1 ordering (SETTLE 64); `log_req` only in RUN before STOP/PF; `cdc_bus` HOLD_CYC per crossing (5/6/14).

---

## 3. WP8 integration plan

**Fixes that must land first:** D1 (WP2 + glue), D2 (WP5). **Before the scenario that needs them:** D3 (S12, any link test), D4 (S5/S6), D5 (S9 with STOP, ABORT tests), D6 (S5 `rtd`), D7 (S6 / key tests), D8 (before S2 READ). **Can wait:** D9–D23 (spec text, tools, B0 timing).

Wiring order (each step compiles in xsim before the next):
1. Clock/reset block: link from `nvme_host_top.link_clk_out`, POR → `rec_rst`, `mig_sys_rst_n`, `core_rst_n`, `ui_rst` synchroniser, `user_rst_out` (D2); MIG generate (`USE_MIG_MODEL`), `calib_tap_*` = 0.
2. `rec_cdc_bridge` + struct assembly `ui_stat_in` / `core_stat_in` (sections 1.3/1.5), `ddr_fill_kib` glue.
3. Text path: uart_rx → parser → rec_ctrl → rec_tx_fmt, debounce, VIO edges, LED, xadc_mon, `tdp_ram_2clk`.
4. ui source path: rec_profile → rec_framer → ddr_fifo → data_ring (`s_data_*`), DTEST latch + `rec_sink_chk` mux (D1).
5. Read path: core tap → vrf_tap → rec_verify; parameters `N_SLOTS/SLOT_BYTES/START_LBA` from one localparam set.
6. XDC (`ax7a200b_rec.xdc`, `_cdc.xdc` incl. `sys_clk_i` via free_clk), `create_project.tcl` sourcing `mig_ip.tcl`, `BLD_DATE`, `hw_status.tcl` labels.

Scenario order in `tb_rec_top` (USE_MIG_MODEL=1, SIM_FAST=1):
1. **S1** (PING/CFG?/TEL) — text path, bridge ages, `!B` after calib.
2. **S9** (MEMTEST, then DTEST) — MIG model + ddr_fifo + sink without the SSD (needs D1; add MEMTEST STOP once D5 is in).
3. **S2** (REC 4 MiB + READ LAST) — full data path both ways, `drain` rule, X6/X15 (needs D8 confirmed).
4. **S4**, **S3** — RT drops/`lost`, profile timing.
5. **S11** (100.3 MHz + jitter) right after S2 — catches CDC/age issues early.
6. **S7**, **S8**, **S10** — SMART during REC, MON/FORMAT, READ END after Format.
7. **S5**, **S6** — PF phases (`t1` needs D4, `rtd` D6, early STOP D7).
8. **S12** + system negatives 9.4 (`calib_fail`, `log_status`, ABORT in RUN) — after D3; add a link-drop case (force `link_lost`) and a "user_clk stopped" case to cover the D3 watchdog.
