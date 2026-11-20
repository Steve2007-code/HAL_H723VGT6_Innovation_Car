@echo off
setlocal EnableExtensions
chcp 936 >nul

rem ====== edit these ======
set "PY=D:\programming\python\python.exe"
set "COM_PORT=COM12"
set "BAUD=115200"
set "CAM=1"
rem 640x480 -> 30fps ; 1280x720 -> only 10fps (usb2 bandwidth limit)
set "RES=640x480"
rem ========================

cd /d "%~dp0"
set "APPDIR="
for /d %%D in ("%~dp0*") do if exist "%%~fD\zones_yolo.py" set "APPDIR=%%~fD"
if not defined APPDIR (
  echo [ERROR] zones_yolo.py not found in any subfolder of this folder.
  pause
  exit /b 1
)
cd /d "%APPDIR%"

if not exist "%PY%" (
  echo [ERROR] Python not found: %PY%
  echo         run  where python  then fix the PY= line above
  pause
  exit /b 1
)

"%PY%" -c "import cv2,numpy,torch,ultralytics,serial" 2>nul
if errorlevel 1 (
  echo [ERROR] This Python is missing packages. Install them first:
  echo.
  echo   "%PY%" -m pip install -r "%~dp0requirements.txt"
  echo   "%PY%" -m pip install torch --index-url https://download.pytorch.org/whl/cpu
  echo.
  pause
  exit /b 1
)

echo.
echo   COM  = %COM_PORT%  @ %BAUD%
echo   CAM  = %CAM%
echo.

"%PY%" zones_yolo.py --weights dial_v3.pt --cam %CAM% --res %RES% --link serial --com %COM_PORT% --baud %BAUD%

echo.
echo [exited] code %errorlevel%
echo If anything looks wrong, screenshot this whole window.
pause
