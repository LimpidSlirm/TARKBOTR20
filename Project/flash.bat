@echo off
rem Rebuild before downloading. Run manually with the intended board connected.
setlocal
if defined KEIL_ROOT (
    set "UV4=%KEIL_ROOT%\UV4\UV4.exe"
) else (
    set "UV4=C:\Keil_v5\UV4\UV4.exe"
)
if not exist "%UV4%" set "UV4=D:\Keil_v5\UV4\UV4.exe"
if not exist "%UV4%" (
    echo [ERROR] UV4.exe not found. Set KEIL_ROOT to your Keil folder.
    exit /b 3
)
call "%~dp0build.bat"
if errorlevel 1 exit /b %errorlevel%
if not exist "%~dp0Build\competition.axf" exit /b 3
if not exist "%~dp0Build\competition.hex" exit /b 3
"%UV4%" -f "%~dp0xproject.uvprojx" -j0 -o "%~dp0flash.log"
exit /b %errorlevel%
