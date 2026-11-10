@echo off
rem ============================================================
rem  Headless flash download (Keil MDK, no GUI needed)
rem  Uses the debugger selected in the project: CMSIS-DAP / DAPLink
rem  Requires: board connected over USB, image already built
rem  Exit code: 0 = OK, non-zero = failed (see flash.log)
rem ============================================================
setlocal
set "UV4=D:\Keil_v5\UV4\UV4.exe"
set "PROJ=%~dp0xproject.uvprojx"
set "LOG=%~dp0flash.log"

if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found: "%UV4%"
    echo         Edit UV4 in this script to your Keil installation path.
    exit /b 3
)

if exist "%LOG%" del /q "%LOG%"

"%UV4%" -f "%PROJ%" -j0 -o "%LOG%"
set "RC=%ERRORLEVEL%"

echo.
echo ------------------------------------------------------------
if %RC% EQU 0 echo [OK]   Download finished.
if %RC% NEQ 0 echo [FAIL] Download failed with code %RC%. Check that the
if %RC% NEQ 0 echo        board is connected and the debugger is correct.
echo Log:  "%LOG%"
echo ------------------------------------------------------------
exit /b %RC%
