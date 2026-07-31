@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo ERROR: .venv was not found. Run setup_windows.bat first.
    pause
    exit /b 1
)

set "NN_CONNECTION=%~1"
if not defined NN_CONNECTION (
    echo Available serial ports:
    ".venv\Scripts\python.exe" -m serial.tools.list_ports
    echo.
    set /p "NN_CONNECTION=Enter connection [COM5]: "
    if not defined NN_CONNECTION set "NN_CONNECTION=COM5"
)

set "NN_BAUD=%~2"
if not defined NN_BAUD set "NN_BAUD=115200"

set "NN_CYCLES=%~3"
if not defined NN_CYCLES set "NN_CYCLES=1000"

echo.
echo Connection : %NN_CONNECTION%
echo Baud       : %NN_BAUD%
echo Cycles     : %NN_CYCLES%
echo.

".venv\Scripts\python.exe" verify_gru_mavlink.py "%NN_CONNECTION%" ^
    --baud %NN_BAUD% ^
    --cycles %NN_CYCLES% ^
    --response-rate 20 ^
    --print-every 50

set "NN_RESULT=%ERRORLEVEL%"
echo.
if "%NN_RESULT%"=="0" (
    echo Verification completed successfully.
) else (
    echo Verification failed with exit code %NN_RESULT%.
    echo Read README_WINDOWS.md for connection and timeout troubleshooting.
)
pause
exit /b %NN_RESULT%
