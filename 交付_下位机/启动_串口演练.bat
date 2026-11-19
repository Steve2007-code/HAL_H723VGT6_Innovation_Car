@echo off
setlocal EnableExtensions
chcp 936 >nul
cd /d "%~dp0"

set "PY=D:\programming\python\python.exe"
set "COM_PORT=COM12"
set "BAUD=115200"

if not exist "%PY%" (
  echo [ERROR] Python not found: %PY%
  pause
  exit /b 1
)

if not exist "%~dp0´®¿ÚÑÝÁ·.py" (
  echo [ERROR] script missing in this folder
  pause
  exit /b 1
)

"%PY%" "%~dp0´®¿ÚÑÝÁ·.py" --com %COM_PORT% --baud %BAUD%

echo.
pause
