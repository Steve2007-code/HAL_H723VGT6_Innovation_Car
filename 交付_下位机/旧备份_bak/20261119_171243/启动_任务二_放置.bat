@echo off
cd /d "%~dp0任务二_放置"

rem ====== 改这几处 ======
set COM_PORT=COM3
set BAUD=115200
set CAM=0
rem ======================

python zones_yolo.py --weights dial_v3.pt --cam %CAM% --link serial --com %COM_PORT% --baud %BAUD%

pause
