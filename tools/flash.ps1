[CmdletBinding()]
param([Parameter(Mandatory = $true)][ValidateSet('F407', 'G3507')][string]$Target)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if ($Target -eq 'F407') {
    if (-not $env:JLINK_ROOT) { throw 'Set JLINK_ROOT to your SEGGER J-Link installation.' }
    $probe = Join-Path $env:JLINK_ROOT 'JLink.exe'
    $elf = Join-Path $root 'firmware/f407/build/Debug/f407.elf'
    if (-not (Test-Path -LiteralPath $probe) -or -not (Test-Path -LiteralPath $elf)) {
        throw 'Set JLINK_ROOT and build F407 first.'
    }
    Push-Location $root
    try {
        & $probe -device STM32F407ZG -if SWD -speed 1000 -autoconnect 1 -NoGui 1 -ExitOnError 1 -CommandFile (Join-Path $PSScriptRoot 'f407_flash.jlink') -Log (Join-Path $root 'firmware/f407/build/Debug/jlink-flash.log')
        if ($LASTEXITCODE -ne 0) { throw 'F407 programming failed; inspect the J-Link log.' }
    } finally { Pop-Location }
} else {
    if (-not $env:OPENOCD_ROOT -or -not $env:OPENOCD_SCRIPTS) {
        throw 'Set OPENOCD_ROOT and OPENOCD_SCRIPTS to your MSPM0-enabled OpenOCD installation.'
    }
    $probe = Join-Path $env:OPENOCD_ROOT 'bin/openocd.exe'
    $elf = Join-Path $root 'firmware/g3507/build-zdt-uart-control/MSPM0.elf'
    if (-not (Test-Path -LiteralPath $probe) -or -not (Test-Path -LiteralPath $elf)) {
        throw 'Set OPENOCD_ROOT and build G3507 first.'
    }
    $config = Join-Path $root 'firmware/g3507/linker/CFG.cfg'
    $elfForOpenOcd = $elf.Replace('\', '/')
    & $probe -s $env:OPENOCD_SCRIPTS -f $config -c "program {$elfForOpenOcd} verify reset exit"
    if ($LASTEXITCODE -ne 0) { throw 'G3507 programming failed; inspect OpenOCD output.' }
}
