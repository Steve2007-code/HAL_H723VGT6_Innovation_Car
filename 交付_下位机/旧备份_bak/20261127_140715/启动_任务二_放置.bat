@echo off
setlocal EnableExtensions
chcp 936 >nul
title 任务二_放置  zones_yolo

rem ==========================================================
rem  任务二（放置）视觉启动器
rem  实际运行目录   新文件\任务二_放置     （视觉组修订版）
rem  实际运行脚本   zones_yolo.py
rem
rem  报文（115200 8N1，ASCII）：
rem    对齐时   双尖括号  N,号码,X,dx,Y,dy          约 20Hz
rem    没对齐   双尖括号  N,0,X,9999,Y,9999         约 10Hz
rem  dx、dy 已含"相机不在爪子正上方"的补偿，下位机只管驱到 0
rem ==========================================================

rem ====== 只改下面这几行 ======
set "PY=D:\programming\python\python.exe"
set "COM_PORT=COM12"
set "BAUD=115200"

rem 相机编号：0=笔记本自带，1=外接 USB 相机。插拔后会变，打不开就换个数
set "CAM=1"

rem 分辨率：1280x720  认盘更准，USB2 下约 10fps
rem         640x480   帧率约 30fps，反馈更快，但盘远 / 小时容易漏检
set "RES=1280x720"

rem 数字怎么读：hybrid=模型+模板互相校验（推荐）
rem             model=只用模型    template=只用模板
set "DIGIT_FROM=hybrid"

rem 置信度门槛：漏检往下调到 0.30，误检往上调到 0.50
set "CONF=0.40"

rem 对准判定：标定点离画面中心多少像素算"对准"
set "TOL=15"

rem 画面太暗 / 偏色时，在引号里填 1 打开对应预处理（可以两个都填）
set "CLAHE="
set "WBGRAY="
rem ============================

set "EXTRA="
if defined CLAHE  set "EXTRA=%EXTRA% --clahe"
if defined WBGRAY set "EXTRA=%EXTRA% --wb-gray"

rem ---- 定位程序目录：优先"新文件"里的修订版，没有才退回根目录老版 ----
set "APPDIR=%~dp0新文件\任务二_放置"
if not exist "%APPDIR%\zones_yolo.py" set "APPDIR=%~dp0任务二_放置"

if not exist "%APPDIR%\zones_yolo.py" (
  echo.
  echo [ERROR] 找不到 zones_yolo.py，找过这两处：
  echo         %~dp0新文件\任务二_放置
  echo         %~dp0任务二_放置
  pause
  exit /b 1
)

if not exist "%APPDIR%\link.py" (
  echo.
  echo [ERROR] 缺 link.py 串口通道模块，zones_yolo.py 里有 from link import Link
  echo         把 新文件\任务一_抓取\link.py 复制进：
  echo         %APPDIR%\
  pause
  exit /b 1
)

if not exist "%APPDIR%\dial_v3.pt" (
  echo.
  echo [ERROR] 缺权重 dial_v3.pt，应该在：
  echo         %APPDIR%\
  pause
  exit /b 1
)

if not exist "%PY%" (
  echo.
  echo [ERROR] 找不到 Python：%PY%
  echo         先跑 where python，再把上面 PY= 那行改对
  pause
  exit /b 1
)

cd /d "%APPDIR%"

"%PY%" -c "import cv2,numpy,torch,ultralytics,serial" 2>nul
if errorlevel 1 (
  echo.
  echo [ERROR] 这个 Python 缺依赖包，先装：
  echo.
  echo   "%PY%" -m pip install -r "%~dp0requirements.txt"
  echo   "%PY%" -m pip install torch --index-url https://download.pytorch.org/whl/cpu
  echo.
  pause
  exit /b 1
)

echo.
echo   程序目录 = %APPDIR%
echo   串口     = %COM_PORT%  @ %BAUD%  8N1
echo   相机     = %CAM%   分辨率 %RES%
echo   读数字   = %DIGIT_FROM%   置信度 %CONF%   对准阈值 %TOL% px
echo   预处理   =%EXTRA%
echo.
echo   有效帧： ^<^<N,号码,X,dx,Y,dy^>^>
echo   无效帧： ^<^<N,0,X,9999,Y,9999^>^>
echo.
echo   按键： q 退出并存偏移   s 存图 + 存偏移
echo          j / l 标定点左 / 右移   i / k 上 / 下移（每下 2 像素）
echo.

"%PY%" -W ignore::SyntaxWarning zones_yolo.py --weights dial_v3.pt --cam %CAM% --res %RES% --conf %CONF% --tol %TOL% --digit-from %DIGIT_FROM% --link serial --com %COM_PORT% --baud %BAUD%%EXTRA%

echo.
echo [exited] code %errorlevel%
echo 有任何异常，把整个窗口截图发出来
pause
