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
rem TASK color code: 1=red 2=yellow 3=blue 4=green 5=black 6=lightblue
rem e.g. "532" = black then blue then yellow ; "4" = green only
set "TASK=4"
set "LINK=serial"
rem free-run mode: send dx/dy as soon as the target is seen (no align wait)
rem   TOL        huge  -> locked always true
rem   MOTION_TOL huge  -> "turntable stopped" check disabled
rem   LOCK_FRAMES   1  -> no 3-frame confirm
set "TOL=2000"
set "MOTION_TOL=100000"
set "LOCK_FRAMES=1"
rem ========================

cd /d "%~dp0"
set "APPDIR="
for /d %%D in ("%~dp0*") do if exist "%%~fD\live_detect.py" set "APPDIR=%%~fD"
if not defined APPDIR (
  echo [ERROR] live_detect.py not found in any subfolder of this folder.
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
echo   TASK = %TASK%
echo   LINK = %LINK%
echo.

"%PY%" live_detect.py --weights best.pt --cam %CAM% --res %RES% --task %TASK% --link %LINK% --com %COM_PORT% --baud %BAUD% --tol %TOL% --motion-tol %MOTION_TOL% --lock-frames %LOCK_FRAMES%

echo.
echo [exited] code %errorlevel%
echo If anything looks wrong, screenshot this whole window.
pause
