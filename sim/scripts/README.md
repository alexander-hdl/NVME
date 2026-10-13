# Simulation (xsim, Vivado 2022.2)

Tools (Windows): `C:\Xilinx\Vivado\2022.2\bin\xvlog.bat`, `xelab.bat`, `xsim.bat`.
From Git Bash: `/c/Xilinx/Vivado/2022.2/bin/xvlog.bat -sv <files>`.

Every testbench lives in `sim/tb/tb_<name>.sv`, top module `tb_<name>`, and must:
- compile with `xvlog -sv -i rtl rtl/nvme_host_pkg.sv <rtl files> <model files> sim/tb/tb_<name>.sv`
- elaborate with `xelab -debug typical tb_<name> -s tb_<name>_sim` (add `-timescale 1ns/1ps`)
- run with `xsim tb_<name>_sim -R`
- end with `$display("TEST PASSED")` or `$fatal` on error; `run_xsim.*` greps for `TEST PASSED`.

Working directory for xsim artifacts: `sim/work/` (git-ignored).
`run_xsim.sh <name>` / `run_xsim.ps1 <name>`: compile+run one test; with no argument run all `sim/tb/tb_*.sv`.
File lists: each test may provide `sim/tb/tb_<name>.f` (one path per line, relative to repo root) listing the
RTL/model files it needs; the script passes the package first, then the .f list, then the tb.
