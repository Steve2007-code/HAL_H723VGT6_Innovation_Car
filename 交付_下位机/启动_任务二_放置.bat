@echo off
setlocal EnableExtensions
chcp 936 >nul
title 任务二_放置  zones_yolo

rem ==========================================================
rem  任务二（放置）视觉启动器
rem  实际运行目录   任务二_放置  （11-20 20:41，最新版）
rem  实际运行脚本   zones_yolo.py
rem  注意：新文件\任务二_放置 是 11-19 的旧快照 —— 符号没反转、只在对准后才发，
rem        别用那份。本脚本优先根目录，仅在根目录缺文件时才回退过去。
rem
rem  报文（115200 8N1，ASCII）：
rem    对齐时   双尖括号  N,号码,X,dx,Y,dy          约 20Hz
rem    没对齐   双尖括号  N,0,X,9999,Y,9999         约 10Hz
rem  dx、dy 已含"相机不在爪子正上方"的补偿，下位机只管驱到 0
rem ==========================================================

rem ====== 只改下面这几行 ======
set "PY=D:\programming\python\python.exe"

rem 串口号：每换一次 USB 口都可能变。本机现在只有 COM21 是真实 USB 串口
rem         不确定就先跑一遍本 bat，报错里会列出当前所有串口
set "COM_PORT=COM26"
set "BAUD=115200"

rem 通道：serial = 走串口发给下位机
rem       print  = 只在窗口里打印报文（板子没接 / 想先验证识别准不准时用）
set "LINK=print"

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

rem ---- 定位程序目录：优先根目录最新版，没有才退回"新文件"里的 11-19 旧快照 ----
set "APPDIR=%~dp0任务二_放置"
if not exist "%APPDIR%\zones_yolo.py" set "APPDIR=%~dp0新文件\任务二_放置"

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

rem ---- 串口预检：先看存不存在，再试能不能真打开 ----
if /i not "%LINK%"=="serial" goto :skipCom

if not defined COM_PORT goto :comMissing

"%PY%" -c "import sys,serial.tools.list_ports as lp;sys.exit(0 if sys.argv[1].upper() in [p.device.upper() for p in lp.comports()] else 2)" %COM_PORT% 2>nul
if errorlevel 2 goto :comMissing

"%PY%" -c "import sys,serial;serial.Serial(sys.argv[1],115200,timeout=0.05).close()" %COM_PORT% 2>nul
if errorlevel 1 goto :comBusy

:skipCom

echo.
echo   程序目录 = %APPDIR%
echo   通道     = %LINK%
echo   串口     = %COM_PORT%  @ %BAUD%  8N1   [%LINK% 模式]
if /i "%LINK%"=="print" echo   注意：print 模式不开串口，也不会真的发报文，只做识别
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

"%PY%" -W ignore::SyntaxWarning zones_yolo.py --weights dial_v3.pt --cam %CAM% --res %RES% --conf %CONF% --tol %TOL% --digit-from %DIGIT_FROM% --link %LINK% --com %COM_PORT% --baud %BAUD%%EXTRA%

echo.
echo [exited] code %errorlevel%
echo 有任何异常，把整个窗口截图发出来
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
echo         （上面如果是空的，说明一个串口都没识别到）
echo         把上面正确的 COM 号填进本文件开头的 COM_PORT=
echo         要是板子压根没接、只想先测识别，把 LINK= 改成 print
pause
exit /b 1


:comBusy
echo.
echo [ERROR] 串口 %COM_PORT% 存在，但打不开 —— 被别的程序占着
echo         挨个检查：串口调试助手 / XCOM / 飞特舵机软件 / Keil 的串口窗口
echo                   还有可能是一个没关干净的旧视觉程序窗口
echo         全部关掉，再重新双击本 bat
pause
exit /b 1
