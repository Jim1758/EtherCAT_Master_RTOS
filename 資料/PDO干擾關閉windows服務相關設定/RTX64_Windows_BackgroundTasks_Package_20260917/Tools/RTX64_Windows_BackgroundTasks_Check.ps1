# RTX64 / EtherCAT dedicated controller
# Verification only. No settings are changed.

$Targets = @(
    [PSCustomObject]@{ Path = "\Microsoft\Windows\Application Experience\"; Name = "Microsoft Compatibility Appraiser" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\Device Information\"; Name = "Device" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\MemoryDiagnostic\"; Name = "RunFullMemoryDiagnostic" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\MemoryDiagnostic\"; Name = "ProcessMemoryDiagnosticEvents" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\.NET Framework\"; Name = ".NET Framework NGEN v4.0.30319 64" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\.NET Framework\"; Name = ".NET Framework NGEN v4.0.30319" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\.NET Framework\"; Name = ".NET Framework NGEN v4.0.30319 64 Critical" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\.NET Framework\"; Name = ".NET Framework NGEN v4.0.30319 Critical" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\SystemRestore\"; Name = "SR" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\Customer Experience Improvement Program\"; Name = "Consolidator" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\DeviceDirectoryClient\"; Name = "RegisterDevicePeriodic24" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\DeviceDirectoryClient\"; Name = "RegisterDeviceLocationRightsChange" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\DeviceDirectoryClient\"; Name = "RegisterDeviceProtectionStateChanged" },
    [PSCustomObject]@{ Path = "\Microsoft\Windows\DeviceDirectoryClient\"; Name = "RegisterUserDevice" }
)

$results = foreach ($t in $Targets) {
    $task = Get-ScheduledTask -TaskPath $t.Path -TaskName $t.Name -ErrorAction SilentlyContinue
    if ($null -eq $task) {
        [PSCustomObject]@{
            TaskPath = $t.Path
            TaskName = $t.Name
            State    = "NOT FOUND"
            Result   = "N/A"
        }
    } else {
        $state = [string]$task.State
        [PSCustomObject]@{
            TaskPath = $t.Path
            TaskName = $t.Name
            State    = $state
            Result   = if ($state -eq "Disabled") { "PASS" } else { "FAIL" }
        }
    }
}

$results | Format-Table -AutoSize

$logInfo = Get-WinEvent -ListLog 'Microsoft-Windows-TaskScheduler/Operational' -ErrorAction SilentlyContinue
$logResult = if ($logInfo -and $logInfo.IsEnabled) { "PASS" } else { "FAIL" }

Write-Host ""
Write-Host ("TaskScheduler Operational Log: " + $logResult)

$failed = @($results | Where-Object { $_.Result -eq "FAIL" })
Write-Host ""
if ($failed.Count -eq 0 -and $logResult -eq "PASS") {
    Write-Host "PASS: All existing target tasks are Disabled." -ForegroundColor Green
} else {
    Write-Host "FAIL: One or more target tasks are still enabled/ready." -ForegroundColor Red
}
