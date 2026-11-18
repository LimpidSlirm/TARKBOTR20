@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /W4 /WX /wd4244 /wd4100 /std:c11 /Od /RTC1 /fp:strict /DAPP_KEY_TURN_TEST=1 /FIhost_platform_manual.h /I"include" /I"..\.." /I"..\..\..\Robot\app" /I"..\..\..\Robot\control" /I"..\..\..\Robot\modules\turn" /I"..\..\..\Robot\modules\ramp" /I"..\..\..\Robot" /I"..\..\..\Driver" "..\..\..\Robot\modules\turn\turn_control.c" "..\..\..\Robot\modules\turn\ax_turn.c" "..\..\..\Robot\control\ax_kinematics.c" "..\..\..\Robot\control\ax_speed.c" "..\..\..\Robot\modules\ramp\ax_motion_sequence.c" "..\..\..\Robot\modules\ramp\ax_ramp.c" "..\..\..\Driver\ax_mpu6050.c" "..\..\..\Driver\ax_encoder_extended.c" "..\..\test_manual_control.c" /Fe:test_manual_control.exe
exit /b %errorlevel%
