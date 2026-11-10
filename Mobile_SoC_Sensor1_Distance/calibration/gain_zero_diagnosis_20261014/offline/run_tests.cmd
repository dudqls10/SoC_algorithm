@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"../../../Core/Inc" gain_zero_harness.c /Fe:gain_zero_harness.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
gain_zero_harness.exe
exit /b %errorlevel%
