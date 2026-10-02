# Native Windows C checks only. No downloads, global PATH changes or MCU access.
[CmdletBinding()]
param(
    [string]$Compiler = $env:DUALECU_HOST_CC,
    [string[]]$Sources,
    [string]$Name = 'smoke'
)
$ErrorActionPreference = 'Stop'
# Preserve explicit native exit-code handling even in PowerShell 7 profiles.
$PSNativeCommandUseErrorActionPreference = $false

if ($Name -notmatch '^[A-Za-z0-9_][A-Za-z0-9_-]{0,63}$' -or
    $Name -match '^(CON|PRN|AUX|NUL|COM[1-9]|LPT[1-9])$') {
    throw 'Invalid host output name: use 1-64 letters, digits, underscore or hyphen; no extension or reserved device name.'
}
if ([string]::IsNullOrWhiteSpace($Compiler)) {
    $Compiler = 'gcc'
}
$cc = Get-Command -Name $Compiler -CommandType Application -ErrorAction SilentlyContinue |
    Select-Object -First 1
if (-not $cc) { throw "Host C compiler not found: $Compiler. Set -Compiler or DUALECU_HOST_CC to native GCC." }
$root = Split-Path -Parent $PSScriptRoot
if (-not $Sources) { $Sources = @(Join-Path $root 'validation\host\smoke.c') }
$resolvedSources = foreach ($source in $Sources) {
    if (-not (Test-Path -LiteralPath $source -PathType Leaf)) { throw "Host C source not found: $source" }
    (Get-Item -LiteralPath $source).FullName
}
$build = Join-Path $root 'build\host'
New-Item -ItemType Directory -Path $build -Force | Out-Null
$exe = Join-Path $build ($Name + '.exe')
# Remove only this output before compiling, so a failed build cannot run stale code.
if (Test-Path -LiteralPath $exe) { Remove-Item -LiteralPath $exe -Force }
Write-Host '[HOST_ONLY] Native C compile/run; not FreeRTOS, MCU build, flash or hardware acceptance.'
Write-Host "Compiler: $($cc.Source)"
$compileArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror') + @($resolvedSources) + @('-o', $exe)
& $cc.Source @compileArgs
if ($LASTEXITCODE -ne 0) { throw "Host C compilation failed: $LASTEXITCODE" }
if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) { throw 'Host C compilation produced no executable.' }
& $exe
if ($LASTEXITCODE -ne 0) { throw "Host program failed: $LASTEXITCODE" }
Write-Host "[HOST_ONLY PASS] $exe"
