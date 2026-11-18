@echo off
rem Full ARM Compiler 5 build of Project/xproject.uvprojx; no hardware access.
rem Output: Project/Build/competition.axf and competition.hex.
setlocal
where pwsh.exe >nul 2>&1
if errorlevel 1 (
    set "PSSHELL=powershell.exe"
) else (
    set "PSSHELL=pwsh.exe"
)
"%PSSHELL%" -NoProfile -ExecutionPolicy Bypass -File "%~dp0build-arm.ps1"
exit /b %errorlevel%
