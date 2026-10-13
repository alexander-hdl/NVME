# fpga/ - Vivado build of `kc705_top` / `nvme_host_top` (XC7K325T-2 FFG900, Vivado 2022.2)

Everything generated lives in `fpga/build/` (git-ignored). Tracked here:

| path | what |
|---|---|
| `scripts/create_project.tcl` | creates the project `fpga/build/nvme_host`, adds `rtl/filelist.f`, creates/imports and generates `pcie_7x_0`, adds the constraints; top = `kc705_top` (default) or `nvme_host_top` (`-tclargs top=nvme_host_top`) |
| `scripts/build.tcl` | batch flow: IP + synthesis (+ implementation, + bitstream) and the reports in `fpga/build/reports/` |
| `scripts/bit_pre_loose_io.tcl` | write_bitstream pre-hook used by `build.tcl ... loose_io` (only needed for `top=nvme_host_top`) |
| `../rtl/board/kc705_top.sv` | board top: wraps `nvme_host_top`, test-data generator, buttons/DIP/LEDs (see below) |
| `tools/check_pattern.py` | PC-side checker of the recorded test pattern (raw image or block device) |
| `ip/pcie_7x_0/pcie_7x_0.xci` | the IP configuration (only the `.xci` is tracked; all output products are regenerated) |
| `constraints/kc705_board.xdc` | KC705 board I/O of `kc705_top`: SYSCLK (`free_clk`, 200 MHz), CPU_RESET, GPIO buttons / DIP / LEDs, false paths of the user I/O, config properties (CFGBVS, CONFIG_VOLTAGE, compress) |
| `constraints/kc705_fmc_m2.xdc` | FMC / PCIe part: `sys_clk` 100 MHz + pins (GBTCLK0 C8/C7), PERST# (C25), GT LOCs, ASYNC_REG false path (KC705 + FMC M.2 adapter) |
| `constraints/kc705_fmc_m2_cdc.xdc` | clock groups, implementation only (the IP's clocks do not exist while the IP is a black box in `synth_design`) |

Every constraint exists exactly once. There is no `data_clk` constraint any more: `kc705_top` feeds `nvme_host_top.data_clk` from `user_clk_out`.

## How to build

```
cd fpga/build            (any directory works, logs go to the current directory)
C:\Xilinx\Vivado\2022.2\bin\vivado.bat -mode batch -source ..\scripts\build.tcl -log build.log -journal build.jou -tclargs synth
```

| `-tclargs` | result |
|---|---|
| `synth` (default) | create project if missing, IP (OOC) synthesis, `synth_1`, `reports/synth_*.rpt`, `<top>_synth.dcp` (~4 min) |
| `impl` | + `impl_1` up to `route_design`, `reports/route_*.rpt`, `<top>_routed.dcp` |
| `bit` | + `write_bitstream` -> `fpga/build/<top>.bit` (`kc705_top.bit`: all pins assigned, no `loose_io` needed) |
| `top=kc705_top` (default) / `top=nvme_host_top` | project top; a different top than the existing project recreates it. `nvme_host_top` has no pins for the user-side ports: its `bit` fails the DRC NSTD-1 / UCIO-1 (that path is untested since the board top was added) |
| `loose_io` (with `bit`) | demote NSTD-1 / UCIO-1 to warnings (bring-up experiments with `top=nvme_host_top` only) |
| `jobs=N`, `recreate` | parallel jobs (default 4); force `create_project.tcl` even if the project exists |

`create_project.tcl` alone (`-source fpga/scripts/create_project.tcl`) only creates the project and generates the IP (~15 s).
It re-uses the tracked `.xci` (via `import_ip`); `-tclargs regen_ip` re-creates the IP from the property list in the
script and rewrites the `.xci` (the result was verified to be byte-identical to the tracked file).

No `rtSynthParallelPrep.tcl` problem was seen on this machine with the project flow (`launch_runs`); the only related message is the
harmless `WARNING [Synth 8-7080] Parallel synthesis criteria is not met`.

## pcie_7x_0 settings (IP `xilinx.com:ip:pcie_7x:3.3`)

Properties are applied in stages with read-back (`ip_stage` in `create_project.tcl`); the order matters because the IP's
GUI rules re-derive dependent parameters (e.g. `Device_Port_Type` resets the class code, `Interface_Width` the user clock).
Names/values were taken from `list_property [get_ips pcie_7x_0]` and `data/ip/xilinx/pcie_7x_v3_3/xgui/pcie_7x_v3_3.tcl`.

| GUI option | `CONFIG.` property | value | note |
|---|---|---|---|
| mode | `mode_selection` | `Advanced` | |
| Device/Port type | `Device_Port_Type` | `Root_Port_of_PCI_Express_Root_Complex` | |
| Block location | `PCIe_Blk_Locn` | `X0Y0` | only block of the XC7K325T |
| Lane width | `Maximum_Link_Width` | `X4` | |
| Link speed | `Link_Speed` | `5.0_GT/s` | |
| Target link speed (Link Control 2) | `Trgt_Link_Speed` | `4'h2` | 5 GT/s |
| Reference clock | `Ref_Clk_Freq` | `100_MHz` | |
| AXI-S width | `Interface_Width` | `128_bit` | |
| User clock | `User_Clk_Freq` | `125` | `user_clk_out` = `userclk2`, 8 ns |
| Max payload | `Max_Payload_Size` | `256_bytes` | |
| Performance level | `Perf_Level` | `High` | |
| Buffering optimised for bus mastering | `Buf_Opt_BMA` | `false` | deliberate, see below |
| Class code | `Class_Code_Base` / `Class_Code_Sub` / `Class_Code_Interface` | `06` / `04` / `00` | generated `class_code = 060400` |
| BAR0 | `Bar0_Enabled` | left `true` (default, 2 KB) | the IP rejects "all BARs disabled" (`At least one Bar must be selected`), also in Root Port mode. The Root Port never programs it and does no BAR filtering |
| Legacy interrupt | `Legacy_Interrupt` | `NONE` | |
| MSI / MSI-X | `MSI_Enabled`, `MSIx_Enabled` | `false`, `false` | |
| ECRC / AER | `AER_Enabled`, `RECRC_Check`, `AER_ECRC_Check_Capable`, `AER_ECRC_Gen_Capable` | `false`, `0`, `false`, `false` | |
| Enable message routing | `En_route_err_cor/nfl/ftl`, `En_route_inta..d`, `En_route_pm_pme`, `En_route_pme_to_ack` | all `false` | |
| Read completion boundary | `RCB` | `64_byte` | generated `c_rcb = 0` |
| Extended tag | `Extended_Tag_Field`, `Extended_Tag_Default` | `true`, `true` | 8-bit tags supported and enabled by default |
| NP flow control | `Receive_NP_Request` | `false` | gives the port `rx_np_ok` (`rx_np_req` stays on the port list, tied to 1) |
| Shared logic | `en_ext_clk`, `shared_logic_in_core` | `false`, `true` | pipe clock (1 MMCM, BUFGs) and GT_COMMON inside the core -> only `sys_clk` is needed |
| DRP ports | `PCIe_Debug_Ports` | `false` | |

Everything else is the IP default. Resulting flow-control credits (generated `c_fc_*`): NP header 12 / data 24, completion
header 36 / data 205 (infinite completions: `c_cpl_inf = TRUE`), core footprint 1812 LUT / 3246 FF / 4 BRAM / 1 MMCM / 7 BUFG.
`Buf_Opt_BMA = true` would shrink the NP header credits to 4 to enlarge the completion buffer; this Root Port is the *target*
of the SSD's many outstanding memory reads (ARCHITECTURE.md 5.x queues 12 NP headers), so the default `false` is used.

The 168 ports of the generated `pcie_7x_0` (`ip/pcie_7x_0/pcie_7x_0.veo`) are all connected by name in
`rtl/pcie/pcie7x_wrapper.sv`; unused outputs are open, unused inputs are tied as in the Xilinx Root Port example design
(`open_example_project`, `cgator_wrapper.v`). The Tcl check "every IP port appears exactly once in the instance" returned no missing and no extra port.

## Board top `kc705_top` (`rtl/board/kc705_top.sv`)

Wraps `nvme_host_top`, so that every port has a pin and `build.tcl bit` needs no `loose_io`. Buttons, DIP switches and LEDs are on the
free-running `link_clk` (they work before the first link-up and after `link_fail`); the test-data generator is on `user_clk` and
`data_clk = user_clk_out` (no separate data clock, no `data_clk` constraint).

| pin (KC705) | port | function |
|---|---|---|
| AD12 / AD11 (LVDS) | `free_clk_p/n` | SYSCLK 200 MHz -> `link_clk`, `LINK_CLK_HZ = 200_000_000` |
| C8 / C7 | `sys_clk_p/n` | PCIe refclk 100 MHz, FMC GBTCLK0_M2C |
| C25 (LVCMOS25) | `pcie_perst_n` | PERST# to the FMC / M.2 slot |
| GTXE2_CHANNEL_X0Y12..15 | `pci_exp_*[3:0]` | PCIe lanes (FMC DP0..3) |
| AB7 (LVCMOS15) | `cpu_reset` | CPU_RESET, active high, inverted to `board_rst_n`; also the way to start a new run |
| G12 (LVCMOS25) | `gpio_sw_c` | START: latches `ctrl_start` |
| AB12 (LVCMOS15) | `gpio_sw_s` | STOP: latches the stop request (after START); `ctrl_stop` follows once the generator has stopped |
| AA12 / AG5 / AC6 (LVCMOS15) | `gpio_sw_n/e/w` | LED pages while held: `status_flags` / {cmd_errors!=0, lba_range_err, link_width[1:0], link_lost, link_fail, link_retries[1:0]} / `status_error` |
| Y29, W29, AA28, Y28 (LVCMOS25) | `gpio_dip_sw[0..3]` | [1:0] rate: full, 1/2, 1/4, 1/8; [3:2] length: 1 GiB, 8 GiB, 64 GiB, unlimited |
| AB8, AA8, AC9, AB9 (LVCMOS15), AE26, G19, E18, F16 (LVCMOS25) | `gpio_led[0..7]` | link_up, gen2, enum_done, nvme_ready, running, done, error (any), heartbeat |

All KC705 user-I/O pins are marked `verify UG810` in `kc705_board.xdc` (from memory of the KC705 master XDC; the DRC only proves that the pins exist and
that the bank voltages are consistent).

Test pattern: beat `n` (32-bit beat counter from 0, wraps after 64 GiB) = `{~n, n ^ 32'hA5A5A5A5, n*7, n}` (128-bit AXI-S, little-endian words in memory), identical to
`sim/tb/tb_top.sv`; the recording starts at LBA 0, beat `n` at byte `16*n`. Check a recorded SSD on a PC with `dd` + `fpga/tools/check_pattern.py`:

```
dd if=/dev/nvme1n1 of=rec.img bs=1M count=1024 iflag=direct
python fpga/tools/check_pattern.py rec.img --expect 1G         # or the device itself: ... /dev/nvme1n1 --bytes 1G
```
It prints the number of verified bytes and the first mismatch (byte offset, LBA, beat, expected / found word); exit status 1 on a mismatch.
`vivado.bat` splits `key=value` tclargs into `key value`; `build.tcl` / `create_project.tcl` merge them back, so `top=nvme_host_top` and `jobs=8` work as written.

## Results (Vivado 2022.2.2, `xc7k325tffg900-2`, default strategies, `kc705_top`, no `loose_io`)

`build.tcl bit` (project re-created, IP + synth + impl + bitstream) finished in 8 min without errors and without critical warnings; `fpga/build/kc705_top.bit` (4.7 MB, compressed) was written.
The DRC of the routed design has no error (NSTD-1 / UCIO-1 pass): REQP-1839 warnings (20 shown, rule limit reached), 1 RTSTAT-10 and 2 REQP-181 advisories, see Open items.

| | LUT | FF | RAMB36 | DSP | BUFG | MMCM | GTX | bonded IOB |
|---|---|---|---|---|---|---|---|---|
| `kc705_top`, after `synth_design` (incl. IP) | 7 864 (3.9 %) | 7 652 (1.9 %) | 270 (60.7 %) | 0 | | | 4 | |
| after `route_design` | 7 257 (3.6 %) | 6 733 (1.7 %) | 270 | 0 | 8 | 1 | 4 | 21 of 500 |

(the 8 x 128 KiB data slots are 256 RAMB36 of the 270; the statistic buses and the data stream are no longer pins, so the IOB count dropped from 355 to 21).

Post-route timing (`reports/route_timing_summary.rpt`, "All user specified timing constraints are met"):

| clock | period | WNS setup | WHS hold |
|---|---|---|---|
| `userclk2` (= `user_clk_out` = `data_clk`, the whole NVMe host and the generator) | 8.000 ns (125 MHz) | **+0.345 ns** (worst of the design) | +0.059 ns |
| `clk_250mhz_mux_x0y0` (IP internal pipe clock) | 4.000 ns | +0.460 ns | +0.067 ns |
| `userclk1` | 4.000 ns | +1.706 ns | +0.033 ns |
| `free_clk` (link_bringup, board logic) | 5.000 ns (200 MHz) | +1.053 ns | +0.145 ns |
| `sys_clk` | 10.000 ns | +9.029 ns | +0.172 ns |

Design totals: WNS +0.345 ns, TNS 0, WHS +0.033 ns, 0 failing endpoints of 29 305. Not constrained by design (check_timing): 10 inputs (CPU_RESET, 5 buttons, 4 DIP) and
9 outputs (8 LEDs, PERST#) without input/output delay; they are `set_false_path` in `kc705_board.xdc` (asynchronous or static). Compared with the bare `nvme_host_top`
(previous result, WNS +0.517 ns) the margin on `userclk2` shrank by 0.17 ns (different placement; the generator is on the same clock).
The `top=nvme_host_top` flow was only checked up to project creation (generated extra XDC with `free_clk` / `data_clk`), not re-run through synthesis.

## Findings made while building (all handled in the repo)

1. **`IBUFDS_GTE2.ODIV2` cannot be used together with the IP.** The IP feeds `sys_clk` (= `IBUFDS_GTE2.O`) to a BUFG
   (`cpllpd_refclk_inst`), and O->BUFG and ODIV2->BUFG share one output path of the IBUFDS_GTE2: with the earlier design
   (ODIV2 -> BUFG -> `link_clk`) `route_design` fails with `Route 35-54 ... IBUFDS_GTE2.O->BUFGCTRL.I0` (tried: all top-half
   BUFG sites with IBUFDS_GTE2_X0Y6, and IBUFDS_GTE2_X0Y2/X0Y3/X0Y4 incl. the BUFG site used by Xilinx's own example, while the unmodified
   Xilinx Root Port example routes). Leaving ODIV2 unconnected makes the same netlist route (verified, then re-verified in the full flow).
   Consequence: `link_clk` (free-running clock of `link_bringup`, needed before `user_clk` exists) now comes from a separate board
   oscillator: new ports `free_clk_p/n` of `nvme_host_top` (IBUFDS -> BUFG), `LINK_CLK_HZ` default 200 MHz (KC705 SYSCLK).
   The `refclk_odiv2` port of `pcie7x_wrapper` is gone. ARCHITECTURE.md (clocking paragraph, 5.12, parameter table) and docs/03..05 now describe the free-running board clock (`free_clk_p/n` -> IBUFDS -> BUFG, `LINK_CLK_HZ = 200_000_000`).
2. Everything else in the wrapper only needed names/ties corrected (see git diff of `rtl/pcie/pcie7x_wrapper.sv`): the IP has no `tx_cmd_err`,
   needs `int_pclk_sel_slave`, `fc_sel` and about 40 more inputs tied, all error inputs are active high.
3. Root Port with the GTs in quad 118 (`X0Y12..15`, FMC HPC) and refclk on FMC `GBTCLK0_M2C` (IBUFDS_GTE2_X0Y6) places and routes
   with the LOCs in `kc705_fmc_m2.xdc` (the hierarchy paths in the commented example of the original XDC were correct and are now active).

## Open items for a real board

* **Pins**: `kc705_fmc_m2.xdc` pins (refclk C8/C7, PERST# C25, GT LOCs X0Y12..15) and all KC705 user-I/O pins in `kc705_board.xdc` (SYSCLK AD12/AD11, CPU_RESET AB7, GPIO_SW_*, GPIO_DIP_SW*, GPIO_LED_*) come from memory/the documents and are not checked against UG810 / the adapter schematic (`verify UG810`). Place/route and the DRC accept them (bank voltages are consistent), but a wrong net would not be noticed. Check especially SYSCLK (LVDS in a 1.5 V bank, input only) and which GPIO_SW pin is N/E/W/C.
* **PERST# polarity**: `pcie_perst_n` is active low; the Opsero FMC M.2 adapter takes an active-high reset (see `link_bringup.sv`, docs/05 5.1.2). One inverter in `kc705_top` is needed for that adapter.
* **REQP-1839 (20+ warnings)**: the `ENARDEN` pins of the data-ring RAMB36 are driven by `user_reset_out_reg` of the IP, a register with an asynchronous reset (per `report_drc`). A reset assertion could corrupt RAM contents / read values; the data is invalid after a reset anyway, but gating the RAM enable with a synchronous reset copy in `sdp_ram` / `data_ring` would remove the warning. RTSTAT-10 (`store_ltssm` in the IP) is harmless.
* **Free-running clock**: SYSCLK is used as `free_clk_p/n`; with another oscillator set `LINK_CLK_HZ` and `free_clk` in the XDC.
* **GT lane order / polarity** on the FMC M.2 adapter and whether the adapter feeds the 100 MHz refclk on `GBTCLK0_M2C`.
* **Timing margin**: user clock WNS is +0.35 ns at 125 MHz with 8 x 128 KiB slots; the IP's `clk_250mhz` path has +0.46 ns. Re-check after any RTL change; first candidates would be the `hostmem`/`data_ring` BRAM output paths.
* **Reset/PERST# timing** was not simulated against the real IP (the IP is only checked by elaboration/synthesis/implementation here); verify `sys_rst_n` release vs. `user_reset_out` and `rx_np_ok` behaviour in hardware (ILA on `pl_ltssm_state`, `user_lnk_up`).
* `Buf_Opt_BMA = false` / NP credits 12: keep in sync with the request FIFO depth in `hostmem_completer`.
* `stat_bytes_done`, `stat_first_error_lba` and the other wide statistic buses are not routed to pins in `kc705_top` (only the LED pages); add a VIO / ILA (or UART) if they are needed on hardware.
* A new run needs CPU_RESET (the START / STOP latches are only cleared by it); after "unlimited" + STOP the last partial ring slot is not on the SSD.
