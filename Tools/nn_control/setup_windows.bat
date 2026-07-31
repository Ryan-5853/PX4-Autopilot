@echo off
setlocal
cd /d "%~dp0"

echo [1/4] Checking Python launcher...
where py >nul 2>nul
if errorlevel 1 (
    echo ERROR: Python launcher "py" was not found.
    echo Install 64-bit Python 3.12 from https://www.python.org/downloads/windows/
    echo Enable "Install launcher for all users" during installation.
    pause
    exit /b 1
)

echo [2/4] Creating isolated Python environment in .venv...
py -3.12 -m venv .venv
if errorlevel 1 (
    echo ERROR: 64-bit Python 3.12 is required but was not found.
    echo List installed interpreters with: py -0p
    pause
    exit /b 1
)

echo [3/4] Installing pinned binary dependencies...
".venv\Scripts\python.exe" -m pip install --upgrade pip
if errorlevel 1 goto :install_failed
".venv\Scripts\python.exe" -m pip install --only-binary=:all: -r requirements-windows.txt
if errorlevel 1 goto :install_failed

echo [4/4] Running deterministic local model self-test...
".venv\Scripts\python.exe" verify_gru_mavlink.py --self-test --cycles 100
if errorlevel 1 (
    echo ERROR: Dependencies installed, but the local model self-test failed.
    pause
    exit /b 1
)

echo.
echo Setup completed successfully.
echo Connected serial ports:
".venv\Scripts\python.exe" -m serial.tools.list_ports
echo.
echo Next: double-click run_verification.bat or run:
echo   run_verification.bat COM5
pause
exit /b 0

:install_failed
echo.
echo ERROR: Dependency installation failed.
echo Check Internet access and retry setup_windows.bat.
pause
exit /b 1
