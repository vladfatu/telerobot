@echo off
cd /d "%~dp0"
powershell -ExecutionPolicy Bypass -File "%~dp0install\setup_windows.ps1"
pause
