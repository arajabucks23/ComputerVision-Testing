# Build the firmware, and flash it with -Flash.
#   powershell -ExecutionPolicy Bypass -File build.ps1          # build only
#   powershell -ExecutionPolicy Bypass -File build.ps1 -Flash   # build + flash + reset
# Tools come from the STM32Cube VS Code extension's bundles (%LOCALAPPDATA%\stm32cube\bundles).
param([switch]$Flash)
# Native tools print notes on stderr; failures are detected by exit code instead.
$ErrorActionPreference = "Continue"

$bundles = Join-Path $env:LOCALAPPDATA "stm32cube\bundles"
function Find-Tool($bundle, $exe) {
    $hit = Get-ChildItem (Join-Path $bundles $bundle) -Recurse -Filter $exe -ErrorAction SilentlyContinue |
           Sort-Object FullName -Descending | Select-Object -First 1
    if (-not $hit) { throw "$exe not found in $bundles\$bundle" }
    return $hit.DirectoryName
}
$env:PATH = (@(
    (Find-Tool "gnu-tools-for-stm32" "arm-none-eabi-gcc.exe"),
    (Find-Tool "cmake" "cmake.exe"),
    (Find-Tool "ninja" "ninja.exe")
) -join ";") + ";" + $env:PATH

$here = $PSScriptRoot
if (-not (Test-Path "$here\st_cubef7\Drivers\STM32F7xx_HAL_Driver\Inc")) {
    Write-Host "ST libraries missing: fetching them (about 63 MB, once)..."
    & "$here\fetch_st_libs.ps1"
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
cmake -S $here -B "$here\build" -G Ninja
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
cmake --build "$here\build"
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

if ($Flash) {
    $prog = Join-Path (Find-Tool "programmer" "STM32_Programmer_CLI.exe") "STM32_Programmer_CLI.exe"
    & $prog -c port=SWD mode=UR -w "$here\build\evac_cam.bin" 0x08000000 -v -rst
    exit $LASTEXITCODE
}
