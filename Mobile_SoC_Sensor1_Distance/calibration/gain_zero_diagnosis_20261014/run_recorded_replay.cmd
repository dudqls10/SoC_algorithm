@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"../../Core/Inc" replay_boot1_guard.c /Fe:replay_boot1_guard.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
replay_boot1_guard.exe boot1_recorded_replay.csv
exit /b %errorlevel%
