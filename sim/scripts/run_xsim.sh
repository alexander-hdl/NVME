#!/usr/bin/env bash
# =============================================================================
# run_xsim.sh — compile + elaborate + run xsim testbenches (Vivado 2022.2).
#
#   run_xsim.sh <name> [<name> ...]   run sim/tb/tb_<name>.sv
#   run_xsim.sh                       run every sim/tb/tb_*.sv
#
# Works from any cwd (repo root is derived from this script's location).
# Artifacts go to sim/work/<name>/ (one directory per test, so tests can be
# run concurrently).  A test passes when its log contains "TEST PASSED" and the
# tool chain / simulator reported no errors.  Exit code is non-zero on failure.
#
# Environment overrides: VIVADO_BIN (default /c/Xilinx/Vivado/2022.2/bin)
# File lists: sim/tb/tb_<name>.f, one path per line relative to the repo root
# (comments with '#', blank lines ignored).  Order of compilation:
#   rtl/nvme_host_pkg.sv, files of the .f list, sim/tb/tb_<name>.sv
# =============================================================================
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
VIVADO_BIN="${VIVADO_BIN:-/c/Xilinx/Vivado/2022.2/bin}"

# Windows tools need Windows-style paths (D:/...); cygpath exists in Git Bash/MSYS
to_win() { if command -v cygpath >/dev/null 2>&1; then cygpath -m "$1"; else echo "$1"; fi; }
ROOT_W="$(to_win "$ROOT")"

XVLOG="$VIVADO_BIN/xvlog.bat"
XELAB="$VIVADO_BIN/xelab.bat"
XSIM="$VIVADO_BIN/xsim.bat"
[ -x "$XVLOG" ] || [ -f "$XVLOG" ] || { echo "xvlog not found at $XVLOG (set VIVADO_BIN)"; exit 2; }

run_one() {
  local name="$1"
  local tb="$ROOT/sim/tb/tb_${name}.sv"
  local flist="$ROOT/sim/tb/tb_${name}.f"
  local work="$ROOT/sim/work/$name"
  local log="$work/run.log"

  if [ ! -f "$tb" ]; then echo "[$name] missing $tb"; return 2; fi
  rm -rf "$work"; mkdir -p "$work"
  cd "$work" || return 2

  local srcs=("$ROOT_W/rtl/nvme_host_pkg.sv")
  if [ -f "$flist" ]; then
    while IFS= read -r line || [ -n "$line" ]; do
      line="${line%%#*}"; line="$(echo "$line" | tr -d '\r' | xargs)"
      [ -z "$line" ] && continue
      srcs+=("$ROOT_W/$line")
    done < "$flist"
  fi
  srcs+=("$ROOT_W/sim/tb/tb_${name}.sv")

  echo "[$name] xvlog ..."
  "$XVLOG" -sv -L work -i "$ROOT_W/rtl" "${srcs[@]}" > "$log" 2>&1
  local rc=$?
  if [ $rc -ne 0 ] || grep -q "^ERROR" "$log"; then
    echo "[$name] COMPILE FAILED (see $log)"; grep -E "ERROR" "$log" | head -20; return 1
  fi

  echo "[$name] xelab ..."
  "$XELAB" -debug typical -timescale 1ns/1ps -L work "tb_${name}" -s "tb_${name}_sim" >> "$log" 2>&1
  rc=$?
  if [ $rc -ne 0 ] || grep -q "^ERROR" "$log"; then
    echo "[$name] ELABORATION FAILED (see $log)"; grep -E "ERROR" "$log" | head -20; return 1
  fi

  echo "[$name] xsim ..."
  "$XSIM" "tb_${name}_sim" -R >> "$log" 2>&1
  rc=$?
  if grep -q "TEST PASSED" "$log" && ! grep -Eq "^(Fatal|FATAL|ERROR)|Error:" "$log"; then
    echo "[$name] PASS"
    return 0
  fi
  echo "[$name] FAIL (rc=$rc, log: $log)"
  grep -E "ERROR|Fatal|FATAL|Error" "$log" | head -20
  tail -5 "$log"
  return 1
}

if [ $# -gt 0 ]; then
  names=("$@")
else
  names=()
  for f in "$ROOT"/sim/tb/tb_*.sv; do
    [ -e "$f" ] || continue
    b="$(basename "$f" .sv)"; names+=("${b#tb_}")
  done
fi

fail=0
for n in "${names[@]}"; do
  run_one "$n" || fail=1
done
if [ $fail -eq 0 ]; then echo "ALL PASSED (${names[*]})"; else echo "SOME TESTS FAILED"; fi
exit $fail
