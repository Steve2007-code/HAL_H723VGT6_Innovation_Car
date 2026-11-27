@echo off
setlocal EnableExtensions
chcp 936 >nul

rem ====== edit these ======
set "PY=D:\programming\python\python.exe"
rem 串口号：换 USB 口就会变。本机只有 COM21 是真实 USB 串口，COM4/COM5 是蓝牙虚拟口
set "COM_PORT=COM21"
set "BAUD=115200"
set "CAM=1"
rem 640x480 -> 30fps ; 1280x720 -> only 10fps (usb2 bandwidth limit)
set "RES=640x480"
rem TASK color code: 1=red 2=yellow 3=blue 4=green 5=black 6=lightblue
rem e.g. "532" = black then blue then yellow ; "4" = green only
set "TASK=4"
rem 通道：serial = 走串口发给下位机
rem       print  = 不开串口、不真发，报文只在窗口左下角显示（先验证识别时用）
set "LINK=print"
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

rem ---- 串口预检：先看存不存在，再试能不能真打开 ----
if /i not "%LINK%"=="serial" goto :skipCom

"%PY%" -c "import sys,serial.tools.list_ports as lp;sys.exit(0 if sys.argv[1].upper() in [p.device.upper() for p in lp.comports()] else 2)" %COM_PORT% 2>nul
if errorlevel 2 goto :comMissing

"%PY%" -c "import sys,serial;serial.Serial(sys.argv[1],115200,timeout=0.05).close()" %COM_PORT% 2>nul
if errorlevel 1 goto :comBusy

:skipCom

echo.
echo   COM  = %COM_PORT%  @ %BAUD%
echo   CAM  = %CAM%
echo   TASK = %TASK%
echo   LINK = %LINK%
if /i "%LINK%"=="print" echo   注意：print 模式不开串口，也不会真的发报文
echo.

"%PY%" live_detect.py --weights best.pt --cam %CAM% --res %RES% --task %TASK% --link %LINK% --com %COM_PORT% --baud %BAUD% --tol %TOL% --motion-tol %MOTION_TOL% --lock-frames %LOCK_FRAMES%

echo.
echo [exited] code %errorlevel%
echo If anything looks wrong, screenshot this whole window.
pause
exit /b 0


:comMissing
echo.
echo [ERROR] 串口 %COM_PORT% 在这台机器上不存在
echo         常见原因：板子没插 / 换了 USB 口（COM 号会跟着变）/ 驱动没装
echo.
echo         当前系统里的串口：
set "PYTHONIOENCODING=gbk"
"%PY%" -c "import serial.tools.list_ports as lp;ps=sorted(lp.comports(),key=lambda p:p.device);print('\n'.join('    ' + p.device.ljust(8) + p.description for p in ps))"
set "PYTHONIOENCODING="
echo         把上面正确的 COM 号填进本文件开头的 COM_PORT=
echo         板子没接、只想先测识别：把 LINK= 改成 print
pause
exit /b 1


:comBusy
echo.
echo [ERROR] 串口 %COM_PORT% 存在，但打不开 —— 被别的程序占着
echo         挨个检查：串口调试助手 / XCOM / 飞特舵机软件 / Keil 的串口窗口
echo                   还有可能是一个没关干净的旧视觉程序窗口
pause
exit /b 1
