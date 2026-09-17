# RTX64 / EtherCAT dedicated controller
# Disable selected Windows background Scheduled Tasks that were correlated with PDO/RX disturbances.
# Run this script from Windows PowerShell "Run as administrator".
# This script does NOT delete tasks. It only disables the listed tasks.
# DeviceDirectoryClient tasks are applied through a temporary SYSTEM scheduled task because Administrator can receive Access Denied.

$ErrorActionPreference = "Stop"

function Wait-ForUserClose {
    Write-Host ""
    Write-Host "--------------------------------------------------" -ForegroundColor Cyan
    Write-Host "執行完成。請確認上方結果。" -ForegroundColor Cyan
    Write-Host "PowerShell 視窗會保持開啟，不需要按 Enter。" -ForegroundColor Yellow
    Write-Host "確認完成後，請由操作人員手動按 X 關閉視窗。" -ForegroundColor Yellow
    Write-Host "--------------------------------------------------" -ForegroundColor Cyan
}

# ---------------------------
# 0. Administrator check / automatic elevation
# ---------------------------
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)

if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    Write-Host ""
    Write-Host "[INFO] Administrator permission is required." -ForegroundColor Yellow

    if ($PSCommandPath -and (Test-Path $PSCommandPath)) {
        Write-Host "[INFO] Requesting Administrator permission automatically..." -ForegroundColor Yellow
        try {
            $argList = @(
                "-NoExit"
                "-NoProfile"
                "-ExecutionPolicy", "Bypass"
                "-File", ('"{0}"' -f $PSCommandPath)
            )

            Start-Process -FilePath "powershell.exe" `
                          -Verb RunAs `
                          -ArgumentList $argList

            Write-Host "[INFO] An elevated PowerShell window has been opened." -ForegroundColor Green
            Write-Host "[INFO] Continue in the new Administrator window." -ForegroundColor Green
            exit 0
        }
        catch {
            Write-Host ""
            Write-Host "ERROR: Automatic elevation failed." -ForegroundColor Red
            Write-Host "Please right-click Windows PowerShell and choose 'Run as administrator'." -ForegroundColor Red
            Write-Host $_.Exception.Message -ForegroundColor DarkRed
            Wait-ForUserClose
            return
        }
    }
    else {
        Write-Host ""
        Write-Host "ERROR: This code was pasted/executed without a script file path." -ForegroundColor Red
        Write-Host "Please save it as RTX64_Windows_BackgroundTasks_OneClick.ps1" -ForegroundColor Red
        Write-Host "and run the PS1 file, or open PowerShell as Administrator first." -ForegroundColor Red
        Wait-ForUserClose
        return
    }
}

Write-Host ""
Write-Host "=== RTX64 Windows Background Task Profile ===" -ForegroundColor Cyan
Write-Host "Starting..." -ForegroundColor Cyan

# ---------------------------
# 1. Keep TaskScheduler Operational logging enabled for future diagnosis
# ---------------------------
try {
    & wevtutil.exe sl Microsoft-Windows-TaskScheduler/Operational /e:true
    Write-Host "[OK] TaskScheduler/Operational logging enabled." -ForegroundColor Green
} catch {
    Write-Host "[WARN] Could not enable TaskScheduler Operational log: $($_.Exception.Message)" -ForegroundColor Yellow
}

# ---------------------------
# 2. Target list
# ---------------------------
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

# ---------------------------
# 3. Build a short helper script and run it as SYSTEM
# ---------------------------
$HelperPath = Join-Path $env:WINDIR "Temp\RTX64_Disable_BackgroundTasks_SYSTEM.ps1"
$HelperTask = "RTX64_Disable_BackgroundTasks_Temp"

$helperLines = @(
    '$ErrorActionPreference = "Continue"',
    'Import-Module ScheduledTasks -ErrorAction SilentlyContinue'
)

foreach ($t in $Targets) {
    $p = $t.Path.Replace("'", "''")
    $n = $t.Name.Replace("'", "''")
    $helperLines += "try { `$x = Get-ScheduledTask -TaskPath '$p' -TaskName '$n' -ErrorAction SilentlyContinue; if (`$null -ne `$x) { Disable-ScheduledTask -TaskPath '$p' -TaskName '$n' -ErrorAction Stop | Out-Null } } catch { }"
}

$helperLines | Set-Content -Path $HelperPath -Encoding Unicode

$startTime = (Get-Date).AddMinutes(5).ToString("HH:mm")
$taskAction = "powershell.exe -NoProfile -ExecutionPolicy Bypass -File `"$HelperPath`""

& schtasks.exe /Create /TN $HelperTask /SC ONCE /ST $startTime /RU SYSTEM /RL HIGHEST /TR $taskAction /F | Out-Host
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Could not create temporary SYSTEM helper task." -ForegroundColor Red
    Wait-ForUserClose
    return
}

& schtasks.exe /Run /TN $HelperTask | Out-Host
if ($LASTEXITCODE -ne 0) {
    Write-Host "ERROR: Could not run temporary SYSTEM helper task." -ForegroundColor Red
    Wait-ForUserClose
    return
}

Start-Sleep -Seconds 5

# ---------------------------
# 4. Verification
# ---------------------------
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

Write-Host ""
Write-Host "=== FINAL CHECK ===" -ForegroundColor Cyan
$results | Format-Table -AutoSize

$logInfo = Get-WinEvent -ListLog 'Microsoft-Windows-TaskScheduler/Operational' -ErrorAction SilentlyContinue
$logResult = if ($logInfo -and $logInfo.IsEnabled) { "PASS" } else { "FAIL" }
Write-Host ""
Write-Host ("TaskScheduler Operational Log: " + $logResult)

$failed = @($results | Where-Object { $_.Result -eq "FAIL" })

# Save verification report to Desktop
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$desktop = [Environment]::GetFolderPath("Desktop")
$report = Join-Path $desktop ("RTX64_BackgroundTasks_Check_" + $stamp + ".txt")

@(
    "RTX64 Windows Background Task Verification"
    "Time: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
    "TaskScheduler Operational Log: $logResult"
    ""
) | Set-Content -Path $report -Encoding UTF8
$results | Format-Table -AutoSize | Out-String | Add-Content -Path $report -Encoding UTF8

Write-Host ""
if ($failed.Count -eq 0 -and $logResult -eq "PASS") {
    Write-Host "==============================================" -ForegroundColor Green
    Write-Host " PASS: All existing target tasks are Disabled." -ForegroundColor Green
    Write-Host "==============================================" -ForegroundColor Green
} else {
    Write-Host "==============================================" -ForegroundColor Red
    Write-Host " FAIL: One or more existing target tasks are not Disabled." -ForegroundColor Red
    Write-Host " Check the FINAL CHECK table above." -ForegroundColor Red
    Write-Host "==============================================" -ForegroundColor Red
}

Write-Host "Verification report: $report"

# ---------------------------
# 5. Cleanup temporary helper
# ---------------------------
& schtasks.exe /Delete /TN $HelperTask /F | Out-Null
Remove-Item $HelperPath -Force -ErrorAction SilentlyContinue

Write-Host ""
Write-Host "Done."
Wait-ForUserClose
