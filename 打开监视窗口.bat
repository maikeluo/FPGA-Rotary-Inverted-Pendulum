@echo off
chcp 65001 >nul
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 goto usepython
py -3 scripts\live_monitor.py
goto finish
:usepython
python scripts\live_monitor.py
:finish
if errorlevel 1 pause
