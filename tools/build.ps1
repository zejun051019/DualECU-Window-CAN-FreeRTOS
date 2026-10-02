[CmdletBinding()]
param([Parameter(Mandatory = $true)][ValidateSet('F407', 'G3507')][string]$Target)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (-not $env:CMAKE_ROOT -or -not $env:NINJA_ROOT) {
    throw 'Set CMAKE_ROOT and NINJA_ROOT to your CMake and Ninja installations.'
}
$cmake = Join-Path $env:CMAKE_ROOT 'bin/cmake.exe'
$ninja = Join-Path $env:NINJA_ROOT 'ninja.exe'
if (-not (Test-Path -LiteralPath $cmake) -or -not (Test-Path -LiteralPath $ninja)) {
    throw 'Set CMAKE_ROOT and NINJA_ROOT to your CMake and Ninja installations.'
}
if ($Target -eq 'F407') {
    if (-not $env:F407_ARM_GCC_ROOT) { throw 'Set F407_ARM_GCC_ROOT to your STM32 Arm GCC installation.' }
    $savedPath = $env:PATH
    try {
        $env:PATH = (Join-Path $env:F407_ARM_GCC_ROOT 'bin') + ';' + $env:PATH
        $source = Join-Path $root 'firmware/f407'
        $build = Join-Path $source 'build/Debug'
        & $cmake -S $source -B $build -G Ninja -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_MAKE_PROGRAM=$ninja"
        if ($LASTEXITCODE -ne 0) { throw 'F407 configure failed.' }
        & $cmake --build $build
        if ($LASTEXITCODE -ne 0) { throw 'F407 build failed.' }
    } finally { $env:PATH = $savedPath }
} else {
    if (-not $env:ARM_GCC_ROOT) { throw 'Set ARM_GCC_ROOT to your MSPM0 Arm GCC installation.' }
    $source = Join-Path $root 'firmware/g3507'
    $build = Join-Path $source 'build-zdt-uart-control'
    & $cmake -S $source -B $build -G Ninja -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_MAKE_PROGRAM=$ninja" "-DARM_GCC_TOOLCHAIN_ROOT_DIR=$env:ARM_GCC_ROOT" -DDUALECU_ENABLE_ZDT_UART_CONTROL=ON -DDUALECU_ENABLE_TEST_FAULT_INJECTION=OFF
    if ($LASTEXITCODE -ne 0) { throw 'G3507 configure failed.' }
    & $cmake --build $build
    if ($LASTEXITCODE -ne 0) { throw 'G3507 build failed.' }
}
