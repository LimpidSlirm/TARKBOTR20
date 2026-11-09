@echo off
rem ============================================================
rem  Headless build (Keil MDK, no GUI needed)
rem  Usage: build.bat            -> incremental build
rem         build.bat rebuild    -> rebuild all
rem  Exit code: 0 = OK, 1 = warnings, 2 = errors, >=3 = fatal
rem ============================================================
setlocal
set "UV4=D:\Keil_v5\UV4\UV4.exe"
set "PROJ=%~dp0xproject.uvprojx"
set "LOG=%~dp0build.log"

set "OPT=-b"
if /i "%~1"=="rebuild" set "OPT=-r"

if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found: "%UV4%"
    echo         Edit UV4 in this script to your Keil installation path.
    exit /b 3
)

if exist "%LOG%" del /q "%LOG%"

"%UV4%" %OPT% "%PROJ%" -j0 -o "%LOG%"
set "RC=%ERRORLEVEL%"

echo.
echo ------------------------------------------------------------
if %RC% EQU 0 echo [OK]   Build succeeded, no warnings.
if %RC% EQU 1 echo [OK]   Build succeeded with warnings.
if %RC% EQU 2 echo [FAIL] Build failed, see log.
if %RC% GEQ 3 echo [FAIL] UV4 returned code %RC%, see log.
echo Log:  "%LOG%"
echo Image: "%~dp0Objects\xproject.hex"
echo ------------------------------------------------------------
exit /b %RC%
