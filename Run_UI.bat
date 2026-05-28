@echo off
cd /d "%~dp0"

where pyw >nul 2>nul
if %errorlevel%==0 (
    start "" pyw -3 "%~dp0launcher\launcher.py"
    exit /b
)

where pythonw >nul 2>nul
if %errorlevel%==0 (
    start "" pythonw "%~dp0launcher\launcher.py"
    exit /b
)

echo Python launcher not found.
echo Install Python 3.11+ and make sure it is available from PATH.
pause
exit /b 1
