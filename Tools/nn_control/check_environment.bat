@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo ERROR: .venv was not found. Run setup_windows.bat first.
    pause
    exit /b 1
)

echo Python:
".venv\Scripts\python.exe" --version
echo.
echo Installed packages:
".venv\Scripts\python.exe" -m pip show numpy pymavlink pyserial
echo.
echo Detected serial ports:
".venv\Scripts\python.exe" -m serial.tools.list_ports -v
echo.
echo Deterministic model check:
".venv\Scripts\python.exe" verify_gru_mavlink.py --self-test --cycles 100
pause
