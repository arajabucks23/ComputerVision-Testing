@echo off
REM Live demo: STM32 camera -> people / fire / obstacle detector, with the board's
REM temperature and humidity in the top-right corner. Press q (or Esc) to quit.
REM Needs: the board powered (USB in CN1) and its Ethernet cable plugged into this PC.
cd /d "%~dp0"
"C:\cvenv\Scripts\python.exe" people_and_fire_detector.py --stm32 %*
pause
