@echo off
setlocal EnableExtensions
chcp 936 >nul
cd /d "%~dp0"

set "PY=D:\programming\python\python.exe"
set "COM_PORT=COM12"

if not exist "%PY%" (
  echo [ERROR] Python not found: %PY%
  pause
  exit /b 1
)

if not exist "%~dp0环境自检.py" (
  echo [ERROR] script missing in this folder
  pause
  exit /b 1
)

"%PY%" "%~dp0环境自检.py" --com %COM_PORT%

echo.
pause
