@echo off
setlocal
chcp 936 >nul
cd /d "%~dp0任务一_抓取"

rem ====== 改这几处 ======
set "PY=D:\programming\python\python.exe"
set "COM_PORT=COM12"
set "BAUD=115200"
set "TASK=532"
set "CAM=0"
set "LINK=print"
rem ======================

if not exist "%PY%" (
  echo [错误] 找不到 Python：%PY%
  echo 用 where python 查实际路径，填到 PY= 后面
  pause
  exit /b 1
)

"%PY%" -c "import cv2,numpy,torch,ultralytics,serial" 2>nul
if errorlevel 1 (
  echo [错误] 这个 Python 缺库，先执行下面两条：
  echo   "%PY%" -m pip install -r "%~dp0requirements.txt"
  echo   "%PY%" -m pip install torch --index-url https://download.pytorch.org/whl/cpu
  pause
  exit /b 1
)

echo 启动 COM=%COM_PORT% CAM=%CAM% TASK=%TASK% LINK=%LINK%
"%PY%" live_detect.py --weights best.pt --cam %CAM% --task %TASK% --link %LINK% --com %COM_PORT% --baud %BAUD%

echo.
echo [已退出] 窗口别关，报错就整屏截图
pause
