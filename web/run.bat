@echo off
rem optional: serve the panel over http on :8080 (file:// also works)
cd /d "%~dp0"
python -m http.server 8080
