@echo off
cd /d "%~dp0任务一_抓取"

rem ====== 改这几处 ======
set COM_PORT=COM3
set BAUD=115200
set TASK=532
set CAM=0
rem ======================

python live_detect.py --weights best.pt --cam %CAM% --task %TASK% --link serial --com %COM_PORT% --baud %BAUD%

pause
