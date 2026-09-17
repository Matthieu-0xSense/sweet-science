@echo off
rem Optional: serve the panel over http (file:// also works).
rem
rem   run.bat          -> http://localhost:8000/
rem   run.bat 8137     -> any other port
rem
rem Not 8080: on this class of machine Windows reserves it (http.sys URL
rem registration + `netsh interface ipv4 show excludedportrange`), and python
rem then dies with WinError 10013 instead of a plain "port in use".
setlocal
set PORT=%1
if "%PORT%"=="" set PORT=8000
cd /d "%~dp0"
rem The panel is only a front end: its data comes from the hub's WebSocket on
rem :8765. Without the hub the page loads fine and the "host" badge stays dark.
netstat -ano | findstr /r /c:":8765 .*LISTENING" >nul
if errorlevel 1 (
  echo.
  echo WARNING: no hub listening on :8765 - the panel will show "host" disconnected.
  echo Start it in another terminal:  cd ..\host ^&^& python boxe_host.py --ble
  echo                          or:  python boxe_host.py --sim   ^(no hardware^)
  echo.
)
echo Serving Boxe AI panel on http://localhost:%PORT%/index.html
start "" http://localhost:%PORT%/index.html
python -m http.server %PORT%
if errorlevel 1 (
  echo.
  echo python exited with an error. If it says WinError 10013, the port is
  echo reserved by Windows - try another one: run.bat 8137
  pause
)
