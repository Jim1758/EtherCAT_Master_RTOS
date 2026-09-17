@echo off
setlocal
cd /d "%~dp0"

echo.
echo ============================================================
echo RTX64 Windows Background Tasks OneClick
echo ============================================================
echo.
echo Administrator permission will be requested.
echo Click Yes on the UAC prompt.
echo The elevated PowerShell will start the OneClick automatically.
echo No Enter key is required.
echo.

powershell.exe -NoProfile -ExecutionPolicy Bypass -Command ^
  "Start-Process powershell.exe -Verb RunAs -ArgumentList '-NoExit -NoProfile -ExecutionPolicy Bypass -File ""%~dp0RTX64_Windows_BackgroundTasks_OneClick.ps1""'"

echo.
echo Elevated PowerShell launched.
echo You may close this CMD window.
echo The elevated PowerShell window will remain open after the script finishes.
echo.
pause

endlocal
