@echo off
rem ===========================================================================
rem  BB GUI launcher  --  double-click to run.
rem
rem    bb_gui.bat                 start normally (pythonw, no console window)
rem    bb_gui.bat --port COM8     preselect the serial port
rem    bb_gui.bat --debug         start with a console window (see tracebacks)
rem    bb_gui.bat --selftest      run the self-test only, no window
rem
rem  If pythonw is used and the GUI fails to start, the traceback is written to
rem  gui_error.log next to this file and also shown in a message box.
rem ===========================================================================
setlocal
cd /d "%~dp0"

rem ---- locate python 3 -------------------------------------------------------
where py >nul 2>nul
if errorlevel 1 where python >nul 2>nul
if errorlevel 1 (
  echo [X] Python 3 not found in PATH.
  echo     Install Python 3 and tick "Add python.exe to PATH", then retry.
  pause
  exit /b 1
)

set "PY=python"
set "PYW=pythonw"
where python  >nul 2>nul || set "PY=py -3"
where pythonw >nul 2>nul || set "PYW=pyw -3"

rem ---- console modes ---------------------------------------------------------
if /i "%~1"=="--debug" (
  shift
  %PY% bb_gui.py %*
  goto :done
)
if /i "%~1"=="--selftest" (
  %PY% bb_gui.py --selftest
  goto :done
)

rem ---- normal: windowless ----------------------------------------------------
start "" %PYW% bb_gui.py %*

:done
endlocal
