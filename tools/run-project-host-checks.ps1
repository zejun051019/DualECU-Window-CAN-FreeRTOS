# Run the existing PC-only C checks from one repository-local entry point.
[CmdletBinding()]
param([string]$Compiler = $env:DUALECU_HOST_CC)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$runner = Join-Path $PSScriptRoot 'run-host-checks.ps1'
$cases = @(
    @{ Name = 'smoke'; Sources = @('validation/host/smoke.c') }
    @{ Name = 'can_protocol'; Sources = @('shared/can_protocol.c', 'tests/host/can_protocol_test.c') }
    @{ Name = 'can_recovery_client'; Sources = @('shared/can_protocol.c', 'shared/can_recovery_client.c', 'tests/host/can_recovery_client_test.c') }
    @{ Name = 'can_recovery_timing'; Sources = @('shared/can_protocol.c', 'shared/can_recovery_client.c', 'tests/host/can_recovery_timing_test.c') }
    @{ Name = 'can_test_frame'; Sources = @('tests/host/can_test_frame_test.c') }
    @{ Name = 'mcan_timestamp'; Sources = @('shared/mcan_timestamp.c', 'tests/host/mcan_timestamp_test.c') }
    @{ Name = 'motor_pulse_train'; Sources = @('firmware/g3507/user/motor_pulse_train.c', 'tests/host/motor_pulse_train_test.c') }
    @{ Name = 'motor_range'; Sources = @('shared/motor_range.c', 'tests/host/motor_range_test.c') }
    @{ Name = 'window_request'; Sources = @('shared/window_request.c', 'tests/host/window_request_test.c') }
    @{ Name = 'app_button'; Sources = @('firmware/f407/App/Src/app_button.c', 'tests/host/app_button_test.c') }
    @{ Name = 'window_state'; Sources = @('shared/can_protocol.c', 'shared/window_state.c', 'tests/host/window_state_test.c') }
    @{ Name = 'window_demo'; Sources = @('shared/motor_range.c', 'shared/window_demo.c', 'shared/zdt_uart_protocol.c', 'shared/can_protocol.c', 'shared/window_state.c', 'tests/host/window_demo_test.c') }
    @{ Name = 'zdt_motor_can'; Sources = @('shared/zdt_motor_can.c', 'tests/host/zdt_motor_can_test.c') }
    @{ Name = 'zdt_uart_protocol'; Sources = @('shared/zdt_uart_protocol.c', 'shared/can_protocol.c', 'shared/window_state.c', 'tests/host/zdt_uart_protocol_test.c') }
)

foreach ($case in $cases) {
    $sources = @($case.Sources | ForEach-Object { Join-Path $root $_ })
    & $runner -Compiler $Compiler -Sources $sources -Name "project_$($case.Name)"
    if ($LASTEXITCODE -ne 0) {
        throw "Host check failed: $($case.Name)"
    }
}

Write-Host "[HOST_ONLY PASS] $($cases.Count)/$($cases.Count) project checks"
