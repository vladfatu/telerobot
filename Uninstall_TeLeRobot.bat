@echo off
cd /d "%~dp0"
powershell -ExecutionPolicy Bypass -File "%~dp0install\uninstall_windows.ps1"
pause
