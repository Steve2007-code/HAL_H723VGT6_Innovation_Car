@echo off
chcp 65001 >nul
cd /d "%~dp0"
echo ============================================
echo   LD19 Viewer 打包脚本
echo ============================================
echo.

python -c "import PyQt5, serial" 2>nul
if errorlevel 1 (
    echo [!] 缺少依赖，请先执行:
    echo     pip install PyQt5 pyserial pyinstaller
    pause
    exit /b 1
)

python -m PyInstaller --noconfirm --onefile --windowed ^
    --name LD19Viewer ^
    --icon assets\icon.ico ^
    ld19_viewer.py

if errorlevel 1 (
    echo.
    echo [x] 打包失败
    pause
    exit /b 1
)

echo.
echo [ok] 输出: dist\LD19Viewer.exe
echo.
pause
