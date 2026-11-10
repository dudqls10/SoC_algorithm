@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Insiders\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cd /d "%~dp0"
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"C:\Users\SongYoungBin\Desktop\Soc\Mobile_SoC_Experiments\Mobile_SoC_Sensor1_Distance\Core\Inc" integration.c /Fe:integration.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
integration.exe
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"C:\Users\SongYoungBin\Desktop\Soc\Mobile_SoC_Experiments\Mobile_SoC_Sensor1_Distance\Core\Inc" cap_only_guard_tests.c /Fe:cap_only_guard_tests.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
cap_only_guard_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /std:c11 /W4 /WX /Od /I"C:\Users\SongYoungBin\Desktop\Soc\Mobile_SoC_Experiments\Mobile_SoC_Sensor1_Distance\Core\Inc" recorded_replay.c /Fe:recorded_replay.exe /link /INCREMENTAL:NO
if errorlevel 1 exit /b 1
recorded_replay.exe input.csv
if errorlevel 1 exit /b 1
exit /b 0
