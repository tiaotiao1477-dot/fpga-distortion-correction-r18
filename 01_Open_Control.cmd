@echo off
setlocal
cd /d "%~dp0"
set "PY=%FPGA_PYTHON%"
if not defined PY set "PY=python"
"%PY%" -X utf8 -B "%~dp0sw\rotation_dashboard_v5\dashboard.py" %*
if errorlevel 1 pause
