@echo off
chcp 65001 >nul
where py >nul 2>nul
if errorlevel 1 goto usepython
py -3 -m pip install pyserial==3.5
goto finish
:usepython
python -m pip install pyserial==3.5
:finish
pause
