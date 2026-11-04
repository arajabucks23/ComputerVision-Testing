@echo off
REM Records 15 s of the live detector window (as you see it) to
REM test_footage\stm32\demo_<date>_<time>.mp4, then closes.
REM For a different length: record_camera_demo.bat 30
cd /d "%~dp0"
set SECONDS=%1
if "%SECONDS%"=="" set SECONDS=15
"C:\cvenv\Scripts\python.exe" people_and_fire_detector.py --stm32 --record %SECONDS%
pause
