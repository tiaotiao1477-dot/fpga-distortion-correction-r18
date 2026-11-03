@echo off
setlocal
cd /d "%~dp0"
set "PY=%FPGA_PYTHON%"
if not defined PY set "PY=python"
"%PY%" -X utf8 -B "%~dp0sw\quality_inspector_v1\app.py" %*
if errorlevel 1 pause
