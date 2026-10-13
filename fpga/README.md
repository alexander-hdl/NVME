# fpga/ - Vivado build of `nvme_host_top` (XC7K325T-2 FFG900, Vivado 2022.2)

Everything generated lives in `fpga/build/` (git-ignored). Tracked here:

| path | what |
|---|---|
| `scripts/create_project.tcl` | creates the project `fpga/build/nvme_host`, adds `rtl/filelist.f`, creates/imports and generates `pcie_7x_0`, adds the constraints |
| `scripts/build.tcl` | batch flow: IP + synthesis (+ implementation, + bitstream) and the reports in `fpga/build/reports/` |
| `scripts/bit_pre_loose_io.tcl` | write_bitstream pre-hook used by `build.tcl ... loose_io` |
| `ip/pcie_7x_0/pcie_7x_0.xci` | the IP configuration (only the `.xci` is tracked; all output products are regenerated) |
| `constraints/kc705_fmc_m2.xdc` | clocks, example pins, example GT LOCs (KC705 + FMC M.2 adapter) |
| `constraints/kc705_fmc_m2_cdc.xdc` | clock groups, implementation only (the IP's clocks do not exist while the IP is a black box in `synth_design`) |

## How to build

```
cd fpga/build            (any directory works, logs go to the current directory)
C:\Xilinx\Vivado\2022.2\bin\vivado.bat -mode batch -source ..\scripts\build.tcl -log build.log -journal build.jou -tclargs synth
```

| `-tclargs` | result |
|---|---|
| `synth` (default) | create project if missing, IP (OOC) synthesis, `synth_1`, `reports/synth_*.rpt`, `nvme_host_top_synth.dcp` (~4 min) |
| `impl` | + `impl_1` up to `route_design`, `reports/route_*.rpt`, `nvme_host_top_routed.dcp` (~12 min in total) |
| `bit` | + `write_bitstream` -> `fpga/build/nvme_host_top.bit`; fails the DRC NSTD-1 / UCIO-1 while the user-side ports have no pins |
| `loose_io` (with `bit`) | demote NSTD-1 / UCIO-1 to warnings so that a bitstream is produced anyway (bring-up experiments only) |
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

## Results (Vivado 2022.2.2, `xc7k325tffg900-2`, default strategies)

Synthesis finished without errors and without critical warnings; implementation routed completely.

| | LUT | FF | RAMB36 | DSP | BUFG | MMCM | GTX |
|---|---|---|---|---|---|---|---|
| `nvme_host_top`, after `synth_design` (incl. IP) | 7 895 (3.9 %) | 7 669 (1.9 %) | 270 (60.7 %) | 0 | 9 | 1 | 4 |
| same, without the IP (`u_core` + glue) | 6 083 | 4 423 | 266 | 0 | | | |
| `pcie_7x_0` alone (OOC) | 1 812 | 3 246 | 4 | 0 | 7 | 1 | 4 |
| after `route_design` | 7 299 (3.6 %) | 6 767 (1.7 %) | 270 | 0 | 8 | 1 | 4 |

(the 8 x 128 KiB data slots are 256 RAMB36 of the 270). Bonded IOB 355 of 500, because the wide statistic buses and the data stream are top-level ports.

Post-route timing (`reports/route_timing_summary.rpt`, "All user specified timing constraints are met"):

| clock | period | WNS setup | WHS hold |
|---|---|---|---|
| `userclk2` (= `user_clk_out`, the whole NVMe host) | 8.000 ns (125 MHz) | **+0.517 ns** (worst of the design) | +0.056 ns |
| `clk_250mhz_mux_x0y0` / `userclk1` (IP internal pipe clocks) | 4.000 ns | +0.528 / +1.580 ns | |
| `free_clk` (link_bringup) | 5.000 ns (200 MHz) | +1.872 ns | +0.144 ns |
| `data_clk` (placeholder) | 8.000 ns | +3.415 ns | +0.181 ns |
| `sys_clk` | 10.000 ns | +9.029 ns | |

Design totals: WNS +0.517 ns, TNS 0, WHS +0.040 ns, 0 failing endpoints of 29 698. A bitstream (`loose_io`) was written successfully (4.7 MB, compressed).
Not constrained (check_timing): 132 inputs without input delay and 196 outputs without output delay (all user-side ports).

## Findings made while building (all handled in the repo)

1. **`IBUFDS_GTE2.ODIV2` cannot be used together with the IP.** The IP feeds `sys_clk` (= `IBUFDS_GTE2.O`) to a BUFG
   (`cpllpd_refclk_inst`), and O->BUFG and ODIV2->BUFG share one output path of the IBUFDS_GTE2: with the earlier design
   (ODIV2 -> BUFG -> `link_clk`) `route_design` fails with `Route 35-54 ... IBUFDS_GTE2.O->BUFGCTRL.I0` (tried: all top-half
   BUFG sites with IBUFDS_GTE2_X0Y6, and IBUFDS_GTE2_X0Y2/X0Y3/X0Y4 incl. the BUFG site used by Xilinx's own example, while the unmodified
   Xilinx Root Port example routes). Leaving ODIV2 unconnected makes the same netlist route (verified, then re-verified in the full flow).
   Consequence: `link_clk` (free-running clock of `link_bringup`, needed before `user_clk` exists) now comes from a separate board
   oscillator: new ports `free_clk_p/n` of `nvme_host_top` (IBUFDS -> BUFG), `LINK_CLK_HZ` default 200 MHz (KC705 SYSCLK).
   The `refclk_odiv2` port of `pcie7x_wrapper` is gone. ARCHITECTURE.md (clocking paragraph, 5.12, parameter table) and docs/ still describe ODIV2 = 50 MHz.
2. Everything else in the wrapper only needed names/ties corrected (see git diff of `rtl/pcie/pcie7x_wrapper.sv`): the IP has no `tx_cmd_err`,
   needs `int_pclk_sel_slave`, `fc_sel` and about 40 more inputs tied, all error inputs are active high.
3. Root Port with the GTs in quad 118 (`X0Y12..15`, FMC HPC) and refclk on FMC `GBTCLK0_M2C` (IBUFDS_GTE2_X0Y6) places and routes
   with the LOCs in `kc705_fmc_m2.xdc` (the hierarchy paths in the commented example of the original XDC were correct and are now active).

## Open items for a real board

* **Pins**: `kc705_fmc_m2.xdc` pins (refclk C8/C7, PERST# C25, SYSCLK AD12/AD11, GT LOCs X0Y12..15) are examples taken from the
  documents, not checked against UG810 / the adapter schematic. Nothing was rejected by place/route, but they are not verified.
  Missing: `board_rst_n` (KC705 CPU_RESET is active high), `ctrl_*`, `status_*`, the data stream and `stat_*` buses (no pins, no I/O standards).
* **`data_clk`**: a placeholder 125 MHz constraint is in the XDC; set the real period, add `set_input_delay` for `s_data_*`.
* **Free-running clock**: `free_clk_p/n` is new; check the real KC705 SYSCLK I/O standard/pins (the example uses LVDS on AD12/AD11) or supply your own oscillator and set `LINK_CLK_HZ`.
* **GT lane order / polarity** on the FMC M.2 adapter and whether the adapter feeds the 100 MHz refclk on `GBTCLK0_M2C`.
* **Timing margin**: user clock WNS is +0.5 ns at 125 MHz with 8 x 128 KiB slots; the IP's `clk_250mhz` path has +0.53 ns. Re-check after any RTL change; first candidates would be the `hostmem`/`data_ring` BRAM output paths.
* **Reset/PERST# timing** was not simulated against the real IP (the IP is only checked by elaboration/synthesis/implementation here); verify `sys_rst_n` release vs. `user_reset_out` and `rx_np_ok` behaviour in hardware (ILA on `pl_ltssm_state`, `user_lnk_up`).
* `Buf_Opt_BMA = false` / NP credits 12: keep in sync with the request FIFO depth in `hostmem_completer`.
* The unrouted-IO situation prevents a clean `write_bitstream`; a board-level wrapper (or `loose_io` for experiments) is needed.
