# ATP I-Temp 960 GB — recorder prototype runs (2026-11-24)

Bitstream `fpga/build/rec/ax7a200b_rec_top.bit` (commit 4f83b83), PC tool `tools/recorder.py`, UART COM3.

| File | What |
|---|---|
| `rec280_full_tel.csv` | `$T` telemetry 1/s, REC of the whole drive: MODE RT, RATE 280MB/s, DDRSZ 32MiB, LEN 0 (stop at capacity) |
| `rec280_full_console.txt` | console of that run with `!S` / `!RR` events |
| `read_full_tel.csv`, `read_full_console.txt` | READ LAST of the same session (`!RV`) |
| `smart_before.txt`, `smart_after.txt` | `SMART?` before the REC and after the READ |

Results: 960.2 GB in 3430 s (279.9 MB/s), drp=0, cer=0, DDR peak 21.6 MiB, max command latency 82 ms;
READ 234 423 104 blocks ok, 0 bad, 897 MB/s. FPGA Tj 70.6 → 74.2 °C, SSD 44.9 → 65.8 °C.
`p_fpga_est_W` is empty in the CSV (power table built after the run); estimate from `fpga/power/power_table.csv`: ~2.85 W.

## 400 MB/s, whole drive (after FORMAT YES, 470 ms)
`rec400_full_*`, `read400_full_*`, `smart_after400.txt`: MODE RT, RATE 400MB/s (step 40000000), DDRSZ 1024MiB.
960.2 GB in 2401 s (399.9 MB/s), drp=0, cer=0, DDR peak 35.6 MiB, max latency 84 ms; READ 234 423 104 blocks ok, 897 MB/s.
FPGA Tj 71 → 75.5 °C, SSD 65 → 74.8 °C (warning threshold 75 °C, SMART warning-time unchanged).
