param(
    [switch]$HostOnly,
    [switch]$ArmOnly,
    [string]$HostCompiler = $env:HOST_CC,
    [string]$ArmCompiler = $env:ARM_CC
)

$ErrorActionPreference = 'Stop'

if ($HostOnly -and $ArmOnly) {
    throw 'Use at most one of -HostOnly and -ArmOnly.'
}

$projectRoot = Split-Path -Parent $PSScriptRoot
$source = Join-Path $projectRoot 'src\arm_marg_baseline.c'
$build = Join-Path $projectRoot 'build'
if (-not $HostCompiler) { $HostCompiler = 'gcc' }
if (-not $ArmCompiler) { $ArmCompiler = 'arm-linux-gnueabihf-gcc' }
$common = @('-std=c99', '-O3', '-Wall', '-Wextra', '-Wpedantic', '-Werror')

if ($HostOnly) { $requiredCompilers = @($HostCompiler) }
elseif ($ArmOnly) { $requiredCompilers = @($ArmCompiler) }
else { $requiredCompilers = @($HostCompiler, $ArmCompiler) }
foreach ($tool in $requiredCompilers) {
    $resolved = Get-Command $tool -ErrorAction SilentlyContinue
    if (-not $resolved -and -not (Test-Path -LiteralPath $tool)) { throw "Compiler not found: $tool" }
}
New-Item -ItemType Directory -Path $build -Force | Out-Null

if (-not $ArmOnly) {
    & $HostCompiler @common $source '-o' (Join-Path $build 'arm_marg_baseline_host.exe') '-lm'
    if ($LASTEXITCODE -ne 0) { throw "Host compilation failed: $LASTEXITCODE" }
}

if (-not $HostOnly) {
    # Static output avoids a preventable user-space libc mismatch with the
    # AC7020C Linux image.  It does not alter floating-point arithmetic.
    $armFlags = @('-mcpu=cortex-a9', '-mfpu=neon-vfpv3', '-mfloat-abi=hard', '-static')
    & $ArmCompiler @common @armFlags $source '-o' (Join-Path $build 'arm_marg_baseline_zynq_arm') '-lm'
    if ($LASTEXITCODE -ne 0) { throw "Cortex-A9 cross-compilation failed: $LASTEXITCODE" }
}

Get-ChildItem -LiteralPath $build -File |
    Where-Object { $_.Name -in @('arm_marg_baseline_host.exe', 'arm_marg_baseline_zynq_arm') } |
    Select-Object Name, Length, LastWriteTime
