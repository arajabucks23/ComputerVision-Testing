# Downloads the parts of ST's STM32CubeF7 package the firmware needs (HAL, CMSIS,
# lwIP, LAN8742 driver), pinned to the versions this project was built and tested
# with. About 63 MB, into st_cubef7/ next to this script. build.ps1 runs it
# automatically when st_cubef7/ is missing.
$ErrorActionPreference = "Continue"
$here = $PSScriptRoot
$dst = Join-Path $here "st_cubef7"
if (Test-Path (Join-Path $dst "Drivers\STM32F7xx_HAL_Driver\Inc")) {
    Write-Host "st_cubef7 already present."
    exit 0
}

$commit = "79165e2"   # STM32CubeF7 commit used for this project
# core.longpaths: lwIP's doxygen docs have very long file names, which break
# Windows' 260-character path limit when the project sits in a deep folder.
# (-c settings are passed on to the submodule clones too.)
git -c core.longpaths=true clone --filter=blob:none --sparse https://github.com/STMicroelectronics/STM32CubeF7.git $dst
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
git -C $dst config core.longpaths true
git -C $dst checkout $commit
git -C $dst sparse-checkout set --no-cone `
    '/.gitmodules' `
    '/Drivers/CMSIS/Include/*' `
    '/Drivers/CMSIS/Device/ST/STM32F7xx/*' `
    '/Drivers/STM32F7xx_HAL_Driver/*' `
    '/Drivers/BSP/Components/lan8742/*' `
    '/Middlewares/Third_Party/LwIP/*' `
    '/Projects/STM32F746ZG-Nucleo/Applications/LwIP/*' `
    '/Projects/STM32F769I_EVAL/Applications/LwIP/LwIP_HTTP_Server_Raw/*'
git -c core.longpaths=true -C $dst submodule update --init --depth 1 `
    Drivers/STM32F7xx_HAL_Driver `
    Drivers/CMSIS/Device/ST/STM32F7xx `
    Drivers/BSP/Components/lan8742 `
    Middlewares/Third_Party/LwIP
exit $LASTEXITCODE
