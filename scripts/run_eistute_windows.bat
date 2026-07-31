@echo off
setlocal EnableExtensions
cd /d "%~dp0.."

powershell.exe -NoProfile -ExecutionPolicy Bypass -File scripts\run_case_windows.ps1 ^
  -Config configs\eistute\article.conf ^
  -Results results\eistute ^
  -Verifier scripts\verify_eistute.py
exit /b %ERRORLEVEL%
