# =============================================================================
# run_xsim.ps1 - compile + elaborate + run xsim testbenches (Vivado 2022.2).
#
#   run_xsim.ps1 <name> [<name> ...]   run sim/tb/tb_<name>.sv
#   run_xsim.ps1                       run every sim/tb/tb_*.sv
#
# Works from any cwd (repo root is derived from this script's location).
# Artifacts go to sim/work/<name>/ (one directory per test).  A test passes when
# its log contains "TEST PASSED" and no ERROR/Fatal line.  Exit code is non-zero
# on failure.  Environment override: $env:VIVADO_BIN (default C:\Xilinx\Vivado\2022.2\bin)
# File lists: sim/tb/tb_<name>.f, one path per line relative to the repo root
# ('#' comments and blank lines ignored).  Compile order: rtl/nvme_host_pkg.sv,
# the .f list, sim/tb/tb_<name>.sv.
# =============================================================================
param([Parameter(ValueFromRemainingArguments = $true)][string[]]$Names)

$ErrorActionPreference = 'Continue'
$Root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$VivadoBin = if ($env:VIVADO_BIN) { $env:VIVADO_BIN } else { 'C:\Xilinx\Vivado\2022.2\bin' }
$Xvlog = Join-Path $VivadoBin 'xvlog.bat'
$Xelab = Join-Path $VivadoBin 'xelab.bat'
$Xsim  = Join-Path $VivadoBin 'xsim.bat'
if (-not (Test-Path $Xvlog)) { Write-Host "xvlog not found at $Xvlog (set VIVADO_BIN)"; exit 2 }

function Invoke-Tool([string]$Exe, [string[]]$ToolArgs, [string]$Log) {
    # run a .bat tool, append stdout+stderr to the log, return the exit code
    $out = & $Exe @ToolArgs 2>&1 | ForEach-Object { $_.ToString() }
    $out | Add-Content -Path $Log -Encoding ascii
    return $LASTEXITCODE
}

function Invoke-Test([string]$Name) {
    $tb = Join-Path $Root "sim\tb\tb_$Name.sv"
    $fl = Join-Path $Root "sim\tb\tb_$Name.f"
    $work = Join-Path $Root "sim\work\$Name"
    $log = Join-Path $work 'run.log'
    if (-not (Test-Path $tb)) { Write-Host "[$Name] missing $tb"; return $false }
    if (Test-Path $work) { Remove-Item -Recurse -Force $work }
    New-Item -ItemType Directory -Force -Path $work | Out-Null
    Set-Location $work
    New-Item -ItemType File -Force -Path $log | Out-Null

    $srcs = @((Join-Path $Root 'rtl\nvme_host_pkg.sv'))
    if (Test-Path $fl) {
        foreach ($line in Get-Content $fl) {
            $l = ($line -replace '#.*$', '').Trim()
            if ($l) { $srcs += (Join-Path $Root $l) }
        }
    }
    $srcs += $tb

    Write-Host "[$Name] xvlog ..."
    $rc = Invoke-Tool $Xvlog (@('-sv', '-L', 'work', '-i', (Join-Path $Root 'rtl')) + $srcs) $log
    if ($rc -ne 0 -or (Select-String -Path $log -Pattern '^ERROR' -Quiet)) {
        Write-Host "[$Name] COMPILE FAILED (see $log)"
        Select-String -Path $log -Pattern 'ERROR' | Select-Object -First 20 | ForEach-Object { $_.Line }
        return $false
    }
    Write-Host "[$Name] xelab ..."
    $rc = Invoke-Tool $Xelab @('-debug', 'typical', '-timescale', '1ns/1ps', '-L', 'work', "tb_$Name", '-s', "tb_${Name}_sim") $log
    if ($rc -ne 0 -or (Select-String -Path $log -Pattern '^ERROR' -Quiet)) {
        Write-Host "[$Name] ELABORATION FAILED (see $log)"
        Select-String -Path $log -Pattern 'ERROR' | Select-Object -First 20 | ForEach-Object { $_.Line }
        return $false
    }
    Write-Host "[$Name] xsim ..."
    $rc = Invoke-Tool $Xsim @("tb_${Name}_sim", '-R') $log
    $passed = Select-String -Path $log -Pattern 'TEST PASSED' -Quiet
    $bad = Select-String -Path $log -Pattern '^(Fatal|FATAL|ERROR)|Error:' -Quiet
    if ($passed -and -not $bad) { Write-Host "[$Name] PASS"; return $true }
    Write-Host "[$Name] FAIL (rc=$rc, log: $log)"
    Select-String -Path $log -Pattern 'ERROR|Fatal|FATAL|Error' | Select-Object -First 20 | ForEach-Object { $_.Line }
    Get-Content $log -Tail 5
    return $false
}

if (-not $Names -or $Names.Count -eq 0) {
    $Names = Get-ChildItem (Join-Path $Root 'sim\tb') -Filter 'tb_*.sv' | ForEach-Object { $_.BaseName.Substring(3) }
}

$fail = 0
foreach ($n in $Names) {
    if (-not (Invoke-Test $n)) { $fail = 1 }
}
if ($fail -eq 0) { Write-Host "ALL PASSED ($($Names -join ' '))" } else { Write-Host 'SOME TESTS FAILED' }
exit $fail
